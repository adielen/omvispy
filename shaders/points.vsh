uniform mat4 u_modelview;
uniform mat4 u_projection;

attribute vec3 a_position;

void main() {
	mat4 transform = u_projection * u_modelview;
	gl_Position = transform * vec4(a_position, 1.0);
	
	gl_PointSize = 2.0;
}