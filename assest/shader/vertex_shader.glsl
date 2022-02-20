#version 120


in vec3 vertexPosition;
in vec3 vertexColor;

uniform float pointSize;
uniform mat4 viewMatrix;

attribute vec4 vertex;
attribute vec4 color;
attribute float pointRowIndex;

varying float pointIdx;
varying vec3 vert;
varying vec3 color;

void main() {
  gl_Position = viewMatrix * vec4( vertexPosition, 1.0 );
  gl_PointSize  = pointSize;
  gl_Color = vertexColor;

  // for use in fragment shader
  pointIdx = pointRowIndex;
  vert = vertex.xyz;
  color = vertexColor;
}
