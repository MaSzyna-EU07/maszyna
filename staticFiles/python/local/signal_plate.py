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
	SIZE = 256
	# field, edge, screws, letters, grain of the field, dust at the bottom and the sides, blur, height of the letters as a
	# part of the plate, width of the edge, radius of the screws; the blur, the edge and the screws in pixels of a plate of 128,
	# drawn bigger by as much as the plate is
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
		# the shade of each pixel around 128: the grain, the same each time for the same name, less the dust. The grain is as
		# coarse as on a plate of 128
		spread = 2 * look["grain"] + 1
		coarse = max(1, size // 128)
		grain = Image.frombytes("L", (size // coarse, size // coarse), random.Random(seed).randbytes((size // coarse) ** 2))
		grain = grain.point(lambda value: 128 - look["grain"] + value * spread // 256).resize((size, size), Image.BILINEAR)
		shade = ImageChops.subtract(grain, self.dust(look))
		return Image.merge("RGB", [ImageChops.add(Image.new("L", (size, size), channel), shade, 1.0, -128) for channel in look["field"]])

	def fitting(self, draw, text, height, width, tall):
		# the biggest letters of the text no wider and no taller than given
		while height > 8:
			box = draw.textbbox((0, 0), text, font=self.font(height))
			if box[2] - box[0] <= width and box[3] - box[1] <= tall:
				break
			height -= 2
		return height

	def index_width(self, draw, index, height):
		# the speed index after the name: one digit up at its right, two as a fraction across the height of the letters
		if not index:
			return 0
		upper = draw.textbbox((0, 0), index[0], font=self.font(height))
		if len(index) == 1:
			return upper[2] - upper[0]
		lower = draw.textbbox((0, 0), index[1], font=self.font(height))
		return upper[2] - upper[0] + self.slash_room(height) + lower[2] - lower[0]

	def slash_room(self, height):
		# between the digits of a fraction, the slash with room on both sides of it
		return round(height * 0.55)

	def draw_index(self, draw, index, x, top, tall, height, ink):
		font = self.font(height)
		box = draw.textbbox((0, 0), index[0], font=font)
		digit, rise = box[2] - box[0], box[3] - box[1]
		draw.text((x - box[0], top - box[1]), index[0], font=font, fill=ink)
		if len(index) > 1:
			# the lower digit right of the upper one, the slash in the room between them: it passes the middle of the height
			# there, under the upper digit and over the lower one, which leave the middle free
			room = self.slash_room(height)
			low = draw.textbbox((0, 0), index[1], font=font)
			draw.text((x + digit + room - low[0], top + tall - (low[3] - low[1]) - low[1]), index[1], font=font, fill=ink)
			stroke = max(1, round(height / 9))
			middle = x + digit + room / 2
			reach = room * 0.75
			draw.line((round(middle - reach), top + tall, round(middle + reach), top), fill=ink, width=stroke)

	def _render(self, state):
		size = self.SIZE
		look = self.LOOKS.get(str(state.get("look", "")).strip().lower(), self.LOOKS["toned"])
		text = str(state.get("name", "")).strip()
		# index: the digits of the speed index, "4" or "12" for 1/2; m: the signal gives shunting signals too
		index = "".join(letter for letter in str(state.get("index", "")) if letter.isdigit())[:2]
		shunting = str(state.get("m", "")).strip().lower() in ("1", "m", "yes", "true")
		image = self.field(look, text + index + ("m" if shunting else ""))
		draw = ImageDraw.Draw(image)
		scale = size / 128
		rim = round(look["rim"] * scale)
		bolt = round(look["bolt"] * scale)
		# with the m the name takes the top of the plate, the m a strip of its own under it, as on textures/tabl/a-m
		plate = round(size * 0.7) if shunting else size
		draw.rectangle((0, 0, size - 1, size - 1), outline=look["edge"], width=rim)
		if shunting:
			draw.rectangle((0, plate - rim, size - 1, plate - 1), fill=look["edge"])
		for x in (rim + 2 * bolt, size - 1 - rim - 2 * bolt):
			for y in (rim + 2 * bolt, plate - 1 - rim - 2 * bolt):
				draw.ellipse((x - bolt, y - bolt, x + bolt, y + bolt), fill=look["screw"])
		room = size - 2 * (rim + 4 * bolt)
		tall = plate - 2 * (rim + 4 * bolt) if shunting else room
		if text:
			# the biggest letters which fit between the screws, with the index after them
			height = int(size * look["letters"])
			while True:
				height = self.fitting(draw, text, height, room, tall)
				small = round(height * 0.46)
				gap = round(height * 0.05) if index else 0
				box = draw.textbbox((0, 0), text, font=self.font(height))
				width = box[2] - box[0] + gap + self.index_width(draw, index, small)
				if width <= room or height <= 8 * scale:
					break
				height -= 2
			font = self.font(height)
			left = (size - width) // 2
			top = (plate - (box[3] - box[1])) // 2
			draw.text((left - box[0], top - box[1]), text, font=font, fill=look["ink"])
			if index:
				self.draw_index(draw, index, left + box[2] - box[0] + gap, top, box[3] - box[1], small, look["ink"])
		if shunting:
			strip = size - plate - rim
			height = self.fitting(draw, "m", int(strip * 1.2), room, round(strip * 0.6))
			font = self.font(height)
			box = draw.textbbox((0, 0), "m", font=font)
			draw.text(((size - (box[2] - box[0])) // 2 - box[0], plate + (strip - (box[3] - box[1])) // 2 - box[1]), "m", font=font, fill=look["ink"])
		if look["soften"] > 0:
			image = image.filter(ImageFilter.GaussianBlur(look["soften"] * scale))
		return image
