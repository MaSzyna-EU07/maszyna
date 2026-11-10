#encoding: UTF-8

# the name plate of a signal, the way the plates of textures/tabl look: a light enamel field with a darker edge, four
# screws and the name in black. A template puts it on the signal through its plate parameter, e.g.
# "make:signal_plate?look=toned&name=Tm12"
#   name - the text of the plate; long ones are drawn smaller
#   look - plain: clean and bright; toned: greyish, a little dusty at the bottom; aged: darker, grainy and dusty.
#          toned when not given

import random
from PIL import ImageChops, ImageDraw, ImageFont, ImageFilter, Image

class signal_plate(abstractscreenrenderer):
	SIZE = 128
	# field, edge, screws, letters, grain of the field, dust at the bottom and the sides, blur, height of the letters as a
	# part of the plate, width of the edge, radius of the screws
	LOOKS = {
		"plain": dict(field=(239, 240, 239), edge=(96, 96, 96), screw=(80, 80, 80), ink=(0, 0, 0), grain=0, dust=0, soften=0.0, letters=0.62, rim=4, bolt=3),
		"toned": dict(field=(212, 212, 206), edge=(140, 140, 136), screw=(125, 125, 120), ink=(28, 28, 28), grain=4, dust=10, soften=0.4, letters=0.52, rim=3, bolt=2),
		"aged": dict(field=(200, 199, 190), edge=(120, 118, 112), screw=(105, 100, 92), ink=(35, 34, 32), grain=9, dust=28, soften=0.6, letters=0.52, rim=3, bolt=2),
	}
	FONTS = ["./fonts/arialnb.ttf", "./fonts/arialbd.ttf", "./fonts/dejavusans-bold.ttf"]

	def __init__(self, lookup_path):
		self.fonts = {}
		self.dusts = {}

	def font(self, size):
		if size not in self.fonts:
			self.fonts[size] = None
			for path in self.FONTS:
				try:
					self.fonts[size] = ImageFont.truetype(path, size)
					break
				except OSError:
					continue
			if self.fonts[size] is None:
				self.fonts[size] = ImageFont.load_default()
		return self.fonts[size]

	def dust(self, look):
		# darker towards the bottom and the sides; the same for every plate of a look, so made once
		key = id(look)
		if key not in self.dusts:
			size = self.SIZE
			image = Image.new("L", (size, size))
			image.putdata([int(look["dust"] * (y / size) ** 2 + look["dust"] * 0.5 * (abs(2 * x - size) / size) ** 4) for y in range(size) for x in range(size)])
			self.dusts[key] = image
		return self.dusts[key]

	def field(self, look, seed):
		size = self.SIZE
		if look["grain"] == 0 and look["dust"] == 0:
			return Image.new("RGB", (size, size), look["field"])
		# the shade of each pixel around 128: the grain, the same each time for the same name, less the dust
		spread = 2 * look["grain"] + 1
		grain = Image.frombytes("L", (size, size), random.Random(seed).randbytes(size * size))
		grain = grain.point(lambda value: 128 - look["grain"] + value * spread // 256)
		shade = ImageChops.subtract(grain, self.dust(look))
		return Image.merge("RGB", [ImageChops.add(Image.new("L", (size, size), channel), shade, 1.0, -128) for channel in look["field"]])

	def _render(self, state):
		size = self.SIZE
		look = self.LOOKS.get(str(state.get("look", "")).strip().lower(), self.LOOKS["toned"])
		text = str(state.get("name", "")).strip()
		image = self.field(look, text)
		draw = ImageDraw.Draw(image)
		rim = look["rim"]
		bolt = look["bolt"]
		draw.rectangle((0, 0, size - 1, size - 1), outline=look["edge"], width=rim)
		for x in (rim + 2 * bolt, size - 1 - rim - 2 * bolt):
			for y in (rim + 2 * bolt, size - 1 - rim - 2 * bolt):
				draw.ellipse((x - bolt, y - bolt, x + bolt, y + bolt), fill=look["screw"])
		if text:
			# the biggest letters which fit between the screws
			room = size - 2 * (rim + 4 * bolt)
			height = int(size * look["letters"])
			while height > 8:
				box = draw.textbbox((0, 0), text, font=self.font(height))
				if box[2] - box[0] <= room and box[3] - box[1] <= room:
					break
				height -= 2
			font = self.font(height)
			box = draw.textbbox((0, 0), text, font=font)
			draw.text(((size - (box[2] - box[0])) // 2 - box[0], (size - (box[3] - box[1])) // 2 - box[1]), text, font=font, fill=look["ink"])
		if look["soften"] > 0:
			image = image.filter(ImageFilter.GaussianBlur(look["soften"]))
		return image
