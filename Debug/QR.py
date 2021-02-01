from pyzbar import pyzbar
from numpy import array
def detectQR(img):
    barcod = pyzbar.decode(img)
    
    barcod = barcod[0]

    b_data = barcod.data.encode("utf-8")
    (x, y, w, h) = barcode.rect

    return [b_data, (x + w/2, y + h/2), array([array([x, y]), array([x + w, y]), array([x + w, y + h]), array([x, y + h])])]

