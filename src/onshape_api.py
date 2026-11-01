"""Onshape API client. Credentials stay in the external file provided by the owner."""
from pathlib import Path
import base64,email.utils,hashlib,hmac,json,secrets
import urllib.parse,urllib.request,urllib.error
BASE='https://cad.onshape.com'

class Client:
    def __init__(self,key_file):
        keys=json.loads(Path(key_file).read_text())
        self.access=keys['accessKey'];self.secret=keys['secretKey']
    def call(self,method,path,data=None,content_type='application/json',query=None):
        qs=urllib.parse.urlencode(query or {})
        nonce=secrets.token_hex(16);date=email.utils.formatdate(usegmt=True)
        msg='\n'.join([method,nonce,date,content_type,path,qs,'']).lower()
        digest=base64.b64encode(hmac.new(self.secret.encode(),msg.encode(),hashlib.sha256).digest()).decode()
        hdr={'Date':date,'On-Nonce':nonce,'Content-Type':content_type,
             'Accept':'application/json',
             'Authorization':f'On {self.access}:HmacSHA256:{digest}'}
        url=BASE+path+('?' + qs if qs else '')
        request=urllib.request.Request(url,data=data,headers=hdr,method=method)
        try:
            with urllib.request.urlopen(request,timeout=180) as response:
                raw=response.read()
                return json.loads(raw) if raw else {}
        except urllib.error.HTTPError as error:
            raise RuntimeError(f'Onshape HTTP {error.code}: '+error.read().decode()[:1500]) from None

def multipart(fields,file):
    boundary='arachne'+secrets.token_hex(12)
    body=bytearray()
    for name,value in fields.items():
        body+=f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode()
    body+=f'--{boundary}\r\nContent-Disposition: form-data; name="file"; filename="{file.name}"\r\nContent-Type: application/step\r\n\r\n'.encode()
    body+=file.read_bytes()+b'\r\n'
    body+=f'--{boundary}--\r\n'.encode()
    return bytes(body),f'multipart/form-data; boundary={boundary}'
