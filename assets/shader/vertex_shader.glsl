#version 330 core


layout (location = 0) in vec3 vertexPosition;
layout (location = 1) in vec3 vertexColor;

uniform float pointSize;
uniform mat4 viewMatrix;

out vec3 fcolor;

void main() {
  gl_Position = viewMatrix * vec4(vertexPosition, 1.0 );
  gl_PointSize  = pointSize;

  // for use in fragment shader
  fcolor = vertexColor;
}
