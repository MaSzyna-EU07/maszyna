#encoding: UTF-8

# the name plate of a signal, the way the plates of textures/tabl look: a light field with a dark edge, four screws and
# the name in black. A template puts it on the signal through its plate parameter, e.g. "make:signal_plate?name=Tm12"
#   name - the text of the plate; long ones are drawn narrower

from PIL import ImageDraw, ImageFont, Image

class signal_plate(abstractscreenrenderer):
	SIZE = 128
	FIELD = (239, 240, 239)
	EDGE = (96, 96, 96)
	SCREW = (80, 80, 80)
	INK = (0, 0, 0)
	FONTS = ["./fonts/arialnb.ttf", "./fonts/arialbd.ttf", "./fonts/dejavusans-bold.ttf"]

	def __init__(self, lookup_path):
		self.fonts = {}

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

	def _render(self, state):
		size = self.SIZE
		image = Image.new("RGB", (size, size), self.FIELD)
		draw = ImageDraw.Draw(image)
		edge = size // 32
		draw.rectangle((0, 0, size - 1, size - 1), outline=self.EDGE, width=edge)
		screw = size // 20
		for x in (edge + screw, size - 1 - edge - screw):
			for y in (edge + screw, size - 1 - edge - screw):
				draw.ellipse((x - screw // 2, y - screw // 2, x + screw // 2, y + screw // 2), fill=self.SCREW)
		text = str(state.get("name", "")).strip()
		if text:
			# the biggest letters which fit between the screws
			room = size - 2 * (edge + 2 * screw)
			height = int(size * 0.62)
			while height > 8:
				box = draw.textbbox((0, 0), text, font=self.font(height))
				if box[2] - box[0] <= room and box[3] - box[1] <= room:
					break
				height -= 2
			font = self.font(height)
			box = draw.textbbox((0, 0), text, font=font)
			draw.text(((size - (box[2] - box[0])) // 2 - box[0], (size - (box[3] - box[1])) // 2 - box[1]), text, font=font, fill=self.INK)
		return image
