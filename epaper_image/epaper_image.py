from PIL import Image, ImageDraw



# I need to daw:
# lines
# texts
# numbers

img = Image.new('1', (152, 296), color=1)
logo_img = Image.open("logo.bmp")
logo_img = logo_img.convert('1')
logo_img.save('logo_1bit.bmp')
logo_img = logo_img.resize((50, 50))

# Image.Image.paste(img, logo_img)

# img.save('test_img.bmp')

d = ImageDraw.Draw(img)

d.text((10, 30), "Hello World", fill=0)
d.line([(10, 10), (10, 20)], fill=0, width=10)

# img = img.transpose(Image.TRANSPOSE)

img.save('test_img.bmp')

# img_bmp = img.tobytes()


# for ele in img_bmp:
#     print(hex(ele))

print('test')