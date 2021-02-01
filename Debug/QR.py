from pyzbar import pyzbar

def detectQR(img):
    barcod = pyzbar.decode(img)
    
    barcod = barcod[0]

    b_data = barcod.data.encode("utf-8")
    (x, y, w, h) = barcode.rect

    return [b_data, (x, y), (x, y + h), (x + w, y + h), (x + w, y), (x + w/2, y + h/2)]

