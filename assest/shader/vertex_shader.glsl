#version 330 core


in vec3 vertexPosition;
in vec3 vertexColor;

uniform float pointSize;
uniform mat4 viewMatrix;

attribute float pointRowIndex;

varying vec3 color;

void main() {
  gl_Position = viewMatrix * vec4( vertexPosition, 1.0 );
  gl_PointSize  = pointSize;

  // for use in fragment shader
  color = vertexColor;
}
