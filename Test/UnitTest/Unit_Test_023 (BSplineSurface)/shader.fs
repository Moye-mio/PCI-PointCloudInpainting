#version 330 core
out vec4 FragColor;
in vec3 Color;
uniform float ourColor;

void main()
{
//	FragColor = vec4(Color, 1.0);
	FragColor = vec4(Color.x + ourColor, Color.y + ourColor, Color.z + ourColor, 1.0);
}

