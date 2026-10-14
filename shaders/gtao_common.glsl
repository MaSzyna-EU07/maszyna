vec2 octahedral_encode(vec3 n)
{
	n /= abs(n.x) + abs(n.y) + abs(n.z);
	vec2 o = n.xy;
	if (n.z < 0.0)
		o = (1.0 - abs(n.yx)) * vec2(n.x >= 0.0 ? 1.0 : -1.0, n.y >= 0.0 ? 1.0 : -1.0);
	return o;
}

vec3 octahedral_decode(vec2 o)
{
	vec3 n = vec3(o, 1.0 - abs(o.x) - abs(o.y));
	float t = clamp(-n.z, 0.0, 1.0);
	n.x += (n.x >= 0.0 ? -t : t);
	n.y += (n.y >= 0.0 ? -t : t);
	return normalize(n);
}
