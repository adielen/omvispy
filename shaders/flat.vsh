uniform mat4 u_modelview;
uniform mat4 u_projection;
uniform vec3 u_lightpos;

attribute vec3 a_position;
attribute vec3 a_normal;

varying float v_color;

void main() {
	vec3 light_vec = normalize(u_lightpos - a_position);
	v_color = 0.4 + 0.3 * dot(light_vec, a_normal);
	
	mat4 transform = u_projection * u_modelview;
	gl_Position = transform * vec4(a_position, 1.0);
}