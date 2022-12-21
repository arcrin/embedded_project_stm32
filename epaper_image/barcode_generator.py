from barcode import Code128
from barcode.writer import ImageWriter

pd_number = "PD-123"

pd_barcode = Code128(pd_number, writer=ImageWriter())


pd_barcode.save("pd_123_code")