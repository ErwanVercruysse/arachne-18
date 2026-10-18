"""Look up parts in the JLCPCB assembly library (stock, LCSC number, basic/extended, price)."""
import json, sys, urllib.request
URL = 'https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList'


def search(q, n=4):
    req = urllib.request.Request(URL, data=json.dumps({'keyword': q, 'currentPage': 1, 'pageSize': n, 'searchSource': 'search'}).encode(),
                                 headers={'Content-Type': 'application/json', 'User-Agent': 'Mozilla/5.0'})
    d = json.load(urllib.request.urlopen(req, timeout=30))
    out = []
    for c in (d.get('data') or {}).get('componentPageInfo', {}).get('list') or []:
        p = c.get('componentPrices') or [{}]
        out.append(dict(lcsc=c.get('componentCode'), mpn=c.get('componentModelEn'), pkg=c.get('componentSpecificationEn'),
                        stock=c.get('stockCount'), lib=c.get('componentLibraryType'), price=p[0].get('productPrice'),
                        desc=(c.get('describe') or '')[:70]))
    return out


if __name__ == '__main__':
    for q in sys.argv[1:]:
        print('==', q)
        for r in search(q):
            print('  ', r['lcsc'], r['mpn'], '|', r['pkg'], '| stock', r['stock'], '|', r['lib'], '| $', r['price'])
