in vec2 f_coords;

layout(location = 0) out vec4 out_color;

#texture (color_tex, 0, RGB)

uniform sampler2D iChannel0;

vec2 barrelDistortion(vec2 f_coords, float amt) {
	vec2 cc = f_coords - 0.5;
	float dist = dot(cc, cc);
	return f_coords + cc * dist * amt;
}

float sat( float t )
{
	return clamp( t, 0.0, 1.0 );
}

float linterp( float t ) {
	return sat( 1.0 - abs( 1.0*t - 1.0 ) );
}

float remap( float t, float a, float b ) {
	return sat( (t - a) / (b - a) );
}

vec4 spectrum_offset( float t ) {
	vec4 ret;
	float lo = step(t,0.5);
	float hi = 1.0-lo;
	float w = linterp( remap( t, 1.0/6.0, 5.0/6.0 ) );
	ret = vec4(lo,1.0,hi, 1.) * vec4(1.0-w, w, 1.0-w, 1.);

	return pow( ret, vec4(1.0/2.2) );
}

const float max_distort = 0.05;
const int num_iter = 10;
const float reci_num_iter_f = 1.0 / float(num_iter);

void main()
{	
	vec2 uv=f_coords.xy;

	vec4 sumcol = vec4(0.0);
	vec4 sumw = vec4(0.0);	
	for ( int i=0; i<num_iter;++i )
	{
		float t = float(i) * reci_num_iter_f;
		vec4 w = spectrum_offset( t );
		sumw += w;
		sumcol += w * texture( iChannel0, barrelDistortion(uv, .6 * max_distort*t ) );
	}
		
	out_color = sumcol / sumw;
}
