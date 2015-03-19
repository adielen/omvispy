import openmesh
import numpy

from vispy import app
from vispy import gloo
from vispy import util

def get_file_content(filename):
	file = open(filename)
	content = file.read()
	file.close()
	return content

class Canvas(app.Canvas):

	def __init__(self, mesh, mode):
		app.Canvas.__init__(self, keys='interactive')

		inf = float('+inf')

		self.title          = 'OpenMesh'
		self.size           = (800, 600)

		self.aspect         = 4.0 / 3.0
		self.longitude      = 0.0
		self.latitude       = 0.0
		self.distance       = 0.0
		self.min            = openmesh.TriMesh.Point(+inf, +inf, +inf)
		self.max            = openmesh.TriMesh.Point(-inf, -inf, -inf)
		self.center         = openmesh.TriMesh.Point(0.0, 0.0, 0.0)
		self.begin_center   = openmesh.TriMesh.Point(0.0, 0.0, 0.0)
		self.end_center     = openmesh.TriMesh.Point(0.0, 0.0, 0.0)
		self.begin_distance = 0.0
		self.end_distance   = 0.0

		self.n = 0.1
		self.f = 50.0
		self.t = 0.5 * self.n
		self.b = -self.t
		self.l = self.b * self.aspect
		self.r = self.t * self.aspect

		# Triangulate mesh
		# -----------------------------------

		if not mesh.is_trimesh():
			trimesh = openmesh.TriMesh();
			for vh in mesh.vertices():
				trimesh.add_vertex(mesh.point(vh))
			for fh in mesh.faces():
				handles = []
				for vh in mesh.fv(fh):
					handles.append(vh)
				trimesh.add_face(handles)
			mesh = trimesh

		# Calculate bounding box
		# -----------------------------------

		for vh in mesh.vertices():
			self.min.minimize(mesh.point(vh))
			self.max.maximize(mesh.point(vh))

		self.center = 0.5 * (self.min + self.max)

		# Calculate distance
		# -----------------------------------

		self.distance = self.default_distance()

		# Set up framebuffer
		# -----------------------------------

		depth = gloo.RenderBuffer((600, 800))
		color = gloo.Texture2D(shape=(600, 800, 4), format='rgba')
		self.framebuffer = gloo.FrameBuffer(color=color, depth=depth)

		# Set up off-screen shader
		# -----------------------------------

		vsh = get_file_content('omvispy/shaders/offscreen.vsh')
		fsh = get_file_content('omvispy/shaders/offscreen.fsh')
		self.offscreen = gloo.Program(vsh, fsh)

		# Triangle data
		# -----------------------------------

		triangles = []

		for fh in mesh.faces():
			for vh in mesh.fv(fh):
				p = mesh.point(vh)
				triangles.append([p[0], p[1], p[2]])

		self.offscreen['a_position'] = triangles

		# Mode: Points
		# -----------------------------------

		if mode == 'points':
			vsh = get_file_content('omvispy/shaders/points.vsh')
			fsh = get_file_content('omvispy/shaders/points.fsh')

			self.onscreen = gloo.Program(vsh, fsh)

			vertices = []

			for vh in mesh.vertices():
				p = mesh.point(vh)
				vertices.append([p[0], p[1], p[2]])

			self.onscreen['a_position'] = vertices
			self.mode = 'points'

		# Mode: Wireframe
		# -----------------------------------

		if mode == 'wireframe':
			vsh = get_file_content('omvispy/shaders/wireframe.vsh')
			fsh = get_file_content('omvispy/shaders/wireframe.fsh')

			self.onscreen = gloo.Program(vsh, fsh)

			vertices = []

			for eh in mesh.edges():
				heh = mesh.halfedge_handle(eh, 0)
				vh1 = mesh.from_vertex_handle(heh)
				vh2 = mesh.to_vertex_handle(heh)
				p1 = mesh.point(vh1)
				p2 = mesh.point(vh2)
				vertices.append([p1[0], p1[1], p1[2]])
				vertices.append([p2[0], p2[1], p2[2]])

			self.onscreen['a_position'] = vertices
			self.mode = 'lines'

		# Mode: Flat
		# -----------------------------------

		if mode == 'flat':
			vsh = get_file_content('omvispy/shaders/flat.vsh')
			fsh = get_file_content('omvispy/shaders/flat.fsh')

			self.onscreen = gloo.Program(vsh, fsh)

			release_normals = False
			if not mesh.has_face_normals():
				mesh.request_face_normals()
				mesh.update_face_normals()
				release_normals = True

			normals = []

			for fh in mesh.faces():
				n = mesh.normal(fh)
				for vh in mesh.fv(fh):
					normals.append([n[0], n[1], n[2]])

			if release_normals:
				mesh.release_face_normals()

			self.onscreen['a_position'] = triangles
			self.onscreen['a_normal'] = normals
			self.mode = 'triangles'

		# Timer initalization
		# -----------------------------------

		self.timer = app.Timer('auto', connect=self.on_timer, start=True)

	def camera_position(self):
		rotation = numpy.identity(4)
		rotation = util.transforms.xrotate(rotation, numpy.degrees(self.latitude))
		rotation = util.transforms.yrotate(rotation, numpy.degrees(self.longitude))

		base = numpy.dot(rotation.transpose(), numpy.array([[0.0],[0.0],[self.distance],[1.0]]))
		return openmesh.TriMesh.Point(base[0][0], base[1][0], base[2][0]) + self.center

	def default_distance(self):
		ymax = self.max[1] - self.center[1]
		zmax = self.max[2] - self.center[2]
		return 2.25 * (ymax + zmax)

	def on_resize(self, event):
		width, height = event.size
		self.aspect = float(width) / float(height)

		self.l = self.b * self.aspect
		self.r = self.t * self.aspect

		self.framebuffer.resize((height, width))

	def on_mouse_press(self, event):
		if event.button == 2:
			x = event.pos[0]
			y = self.size[1] - event.pos[1]

			self.framebuffer.activate()
			pixels = gloo.wrappers.read_pixels(viewport=(x, y, 1, 1), out_type='float')
			self.framebuffer.deactivate()

			# Unpack depth value
			r = pixels[0][0][0] * 1.0/(256.0*256.0*256.0)
			g = pixels[0][0][1] * 1.0/(256.0*256.0)
			b = pixels[0][0][2] * 1.0/(256.0)
			a = pixels[0][0][3] * 1.0

			depth = r + g + b + a

			self.begin_center = 1.0 * self.center
			self.begin_distance = self.distance

			if depth >= 1.0:
				self.end_center = 0.5 * (self.min + self.max)
				self.end_distance = self.distance
			else:
				zndc = depth * 2.0 - 1.0
				yndc = float(y) / self.size[1] * 2.0 - 1.0
				xndc = float(x) / self.size[0] * 2.0 - 1.0

				zeye = 2.0 * self.f * self.n / (zndc * (self.f - self.n) - (self.f + self.n))
				yeye = -zeye * (yndc * (self.t - self.b) + (self.t + self.b)) / (2.0 * self.n)
				xeye = -zeye * (xndc * (self.r - self.l) + (self.r + self.l)) / (2.0 * self.n)

				position = numpy.array([[xeye], [yeye], [zeye], [1.0]])

				invmat = numpy.identity(4)
				invmat = util.transforms.translate(invmat, 0.0, 0.0, self.distance)
				invmat = util.transforms.xrotate(invmat, numpy.degrees(self.latitude))
				invmat = util.transforms.yrotate(invmat, numpy.degrees(self.longitude))
				invmat = util.transforms.translate(invmat, self.center[0], self.center[1], self.center[2])

				position = numpy.dot(invmat.transpose(), position)

				self.end_center = openmesh.TriMesh.Point(position[0][0], position[1][0], position[2][0])
				self.end_distance = 0.5 * (self.end_center - self.camera_position()).length()

			self.update()

	def on_mouse_wheel(self, event):
		self.distance += event.delta[1] * self.distance * 0.1
		self.begin_distance = self.distance
		self.end_distance = self.distance
		self.update()

	def on_mouse_move(self, event):
		if event.is_dragging:
			x0, y0 = event.last_event.pos
			x1, y1 = event.pos
			dx, dy = x1 - x0, y1 - y0
			width, height = self.size

			# Left click
			# -----------------------------------

			if event.button == 1:
				self.longitude += 2.5 * float(dx) / float(width)
				self.latitude  += 2.5 * float(dy) / float(height)

			# Middle click
			# -----------------------------------

			if event.button == 3:
				rotation = numpy.identity(4)
				rotation = util.transforms.xrotate(rotation, numpy.degrees(self.latitude))
				rotation = util.transforms.yrotate(rotation, numpy.degrees(self.longitude))

				view  = numpy.dot(rotation.transpose(), numpy.array([[0.0],[0.0],[1.0],[1.0]]))[0:3]
				up    = numpy.dot(rotation.transpose(), numpy.array([[0.0],[1.0],[0.0],[1.0]]))[0:3]
				right = numpy.cross(view, up, axis=0)

				up_vec = openmesh.TriMesh.Normal(up[0][0], up[1][0], up[2][0])
				right_vec = openmesh.TriMesh.Normal(right[0][0], right[1][0], right[2][0])

				self.center += self.distance * float(dx) / float(width) * right_vec.normalize()
				self.center += self.distance * float(dy) / float(height) * up_vec.normalize()

		self.update()

	def on_timer(self, event):
		remaining = self.end_center - self.center

		update_center = 0.1 * (self.end_center - self.begin_center)
		update_distance = 0.1 * (self.end_distance - self.begin_distance)

		if remaining.length() > update_center.length():
			self.center += update_center
			self.distance += update_distance
		else:
			self.center = self.end_center
			self.distance = self.end_distance

		self.update()

	def on_draw(self, event):

		# Transforms
		# -----------------------------------

		view = numpy.identity(4)
		view = util.transforms.translate(view, -self.center[0], -self.center[1], -self.center[2])
		view = util.transforms.yrotate(view, -numpy.degrees(self.longitude))
		view = util.transforms.xrotate(view, -numpy.degrees(self.latitude))
		view = util.transforms.translate(view, 0.0, 0.0, -self.distance)

		projection = util.transforms.frustum(self.l, self.r, self.b, self.t, self.n, self.f)

		# On-screen rendering
		# -----------------------------------

		gloo.wrappers.set_state(depth_test=True)

		gloo.set_viewport(0, 0, *self.size)
		gloo.clear((1.0, 1.0, 1.0, 1.0))

		self.onscreen['u_modelview'] = view
		self.onscreen['u_projection'] = projection

		if self.mode == 'triangles':
			lightpos = self.camera_position()
			self.onscreen['u_lightpos'] = [lightpos[0], lightpos[1], lightpos[2]]

		self.onscreen.draw(self.mode)

		# Off-screen rendering
		# -----------------------------------

		self.framebuffer.activate()

		gloo.wrappers.set_state(depth_test=True)
		gloo.set_state(blend=False)

		gloo.set_viewport(0, 0, *self.size)
		gloo.clear((1.0, 1.0, 1.0, 1.0))

		self.offscreen['u_modelview'] = view
		self.offscreen['u_projection'] = projection

		self.offscreen.draw()

		self.framebuffer.deactivate()

def render(mesh, mode):
	canvas = Canvas(mesh, mode)
	canvas.show()
	canvas.app.run()
