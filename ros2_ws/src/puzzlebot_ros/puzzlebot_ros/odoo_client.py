#!/usr/bin/env python3

from __future__ import annotations 

import logging
import os
import threading
import xmlrpc.client
from typing import Callable

logger = logging.getLogger(__name__)


QR_TO_CUSTOMER: dict[str, str] = {
    'popsi':  os.getenv('ODOO_CUSTOMER_POPSI',  'Popsi'),
    'wolmar': os.getenv('ODOO_CUSTOMER_WOLMAR', 'Wolmar'),
    'emezon': os.getenv('ODOO_CUSTOMER_EMEZON', 'Emezon'),
    'logop':  os.getenv('ODOO_CUSTOMER_POPSI',  'Popsi'),
    'logow':  os.getenv('ODOO_CUSTOMER_WOLMAR', 'Wolmar'),
    'logoe':  os.getenv('ODOO_CUSTOMER_EMEZON', 'Emezon'),
}


def _load_env(env_path: str) -> None:
    if not os.path.exists(env_path):
        return
    with open(env_path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#') and '=' in line:
                key, _, val = line.partition('=')
                os.environ.setdefault(key.strip(), val.strip())


class OdooClient:


    def __init__(self, url: str, db: str, user: str, password: str):
        self._url      = url.rstrip('/')
        self._db       = db
        self._user     = user
        self._password = password
        self._uid: int | None = None
        self._auth_lock = threading.Lock()

    @classmethod
    def from_env(cls, env_path: str | None = None) -> 'OdooClient':
        if env_path:
            _load_env(env_path)
        url  = os.getenv('ODOO_URL',      '')
        db   = os.getenv('ODOO_DB',       '')
        user = os.getenv('ODOO_USER',     '')
        pw   = os.getenv('ODOO_PASSWORD', '')
        return cls(url, db, user, pw)

    @property
    def configured(self) -> bool:
        return bool(self._url and self._db and self._user and self._password)


    def _authenticate(self) -> bool:
        with self._auth_lock:
            if self._uid is not None:
                return True
            try:
                common = xmlrpc.client.ServerProxy(
                    f'{self._url}/xmlrpc/2/common', allow_none=True)
                uid = common.authenticate(self._db, self._user, self._password, {})
                if uid:
                    self._uid = uid
                    logger.info(f'Odoo: autenticado uid={uid}')
                    return True
                logger.error('Odoo: autenticación falló (uid=False)')
                return False
            except Exception as exc:
                logger.error(f'Odoo: error de conexión en auth: {exc}')
                return False

    def _models(self) -> xmlrpc.client.ServerProxy:
        return xmlrpc.client.ServerProxy(
            f'{self._url}/xmlrpc/2/object', allow_none=True)

    def _call(self, model: str, method: str, *args, **kw):
        return self._models().execute_kw(
            self._db, self._uid, self._password,
            model, method, list(args), kw)


    def create_delivery(self, qr_dest: str,
                        product_name: str | None = None,
                        src_location: str | None = None) -> int | None:

        if not self.configured:
            logger.warning('Odoo: no configurado — omitiendo create_delivery')
            return None
        if not self._authenticate():
            return None

        customer_name = QR_TO_CUSTOMER.get(qr_dest.lower(), qr_dest.capitalize())
        product_name  = product_name  or os.getenv('ODOO_PRODUCT',      'Beverage Pallet')
        src_location  = src_location  or os.getenv('ODOO_SRC_LOCATION', 'RA/Stock')

        try:
            partners = self._call(
                'res.partner', 'search_read',
                [('name', 'ilike', customer_name)],
                fields=['id', 'name'], limit=1)
            if not partners:
                logger.error(f'Odoo: cliente no encontrado: {customer_name}')
                return None
            partner_id = partners[0]['id']

            prods = self._call(
                'product.product', 'search_read',
                [('name', 'ilike', product_name)],
                fields=['id', 'name', 'uom_id'], limit=1)
            if not prods:
                logger.error(f'Odoo: producto no encontrado: {product_name}')
                return None
            prod     = prods[0]
            prod_id  = prod['id']
            uom_id   = prod['uom_id'][0]

            locs = self._call(
                'stock.location', 'search_read',
                [('complete_name', 'ilike', src_location),
                 ('usage', '=', 'internal')],
                fields=['id', 'name'], limit=1)
            if not locs:
                logger.error(f'Odoo: ubicación no encontrada: {src_location}')
                return None
            src_loc_id = locs[0]['id']

            pts = self._call(
                'stock.picking.type', 'search_read',
                [('code', '=', 'outgoing')],
                fields=['id', 'name', 'default_location_dest_id'], limit=1)
            if not pts:
                logger.error('Odoo: no hay picking type outgoing')
                return None
            pt            = pts[0]
            pt_id         = pt['id']
            dest_loc_id   = pt['default_location_dest_id'][0]

            picking_id = self._call('stock.picking', 'create', {
                'partner_id':       partner_id,
                'picking_type_id':  pt_id,
                'location_id':      src_loc_id,
                'location_dest_id': dest_loc_id,
                'origin':           f'Puzzlebot E80 → {customer_name}',
            })

            self._call('stock.move', 'create', {
                'description_picking': prod['name'],
                'picking_id':          picking_id,
                'product_id':          prod_id,
                'product_uom':         uom_id,
                'product_uom_qty':     1.0,
                'location_id':         src_loc_id,
                'location_dest_id':    dest_loc_id,
            })

            self._call('stock.picking', 'action_confirm', [picking_id])

            logger.info(f'Odoo: delivery creada picking_id={picking_id} '
                        f'cliente={customer_name}')
            return picking_id

        except Exception as exc:
            logger.error(f'Odoo: error en create_delivery: {exc}')
            return None

    def validate_delivery(self, picking_id: int) -> bool:
        if not self.configured:
            return False
        if not self._authenticate():
            return False
        try:
            picking = self._call(
                'stock.picking', 'read', [picking_id],
                fields=['state', 'move_line_ids', 'move_ids'])[0]

            if picking['state'] == 'done':
                return True

            ml_ids = picking.get('move_line_ids', [])
            if ml_ids:
                self._call('stock.move.line', 'write', ml_ids,
                           {'qty_done': 1.0})
            else:
                p = self._call('stock.picking', 'read', [picking_id],
                               fields=['location_id', 'location_dest_id', 'move_ids'])[0]
                for mv_id in p['move_ids']:
                    mv = self._call('stock.move', 'read', [mv_id],
                                   fields=['product_id', 'product_uom',
                                           'product_uom_qty'])[0]
                    self._call('stock.move.line', 'create', {
                        'picking_id':      picking_id,
                        'move_id':         mv_id,
                        'product_id':      mv['product_id'][0],
                        'product_uom_id':  mv['product_uom'][0],
                        'qty_done':        mv['product_uom_qty'],
                        'location_id':     p['location_id'][0],
                        'location_dest_id': p['location_dest_id'][0],
                    })

            
            self._call('stock.picking', 'button_validate', [picking_id])
            logger.info(f'Odoo: picking_id={picking_id} validado')
            return True

        except Exception as exc:
            logger.error(f'Odoo: error en validate_delivery: {exc}')
            return False


    def create_delivery_async(self, qr_dest: str,
                              on_done: Callable[[int | None, str], None] | None = None):
        def _run():
            pid = self.create_delivery(qr_dest)
            if on_done:
                if pid:
                    on_done(pid, f'CREATED:{pid}')
                else:
                    on_done(None, 'ERROR:create_failed')
        t = threading.Thread(target=_run, daemon=True)
        t.start()

    def validate_delivery_async(self, picking_id: int,
                                on_done: Callable[[bool, str], None] | None = None):
        def _run():
            ok = self.validate_delivery(picking_id)
            if on_done:
                if ok:
                    on_done(True, f'DONE:{picking_id}')
                else:
                    on_done(False, 'ERROR:validate_failed')
        t = threading.Thread(target=_run, daemon=True)
        t.start()
