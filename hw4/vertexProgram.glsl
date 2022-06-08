/* Vertex shader for the Phong shading implementation. Calculates the 
 * per-vertex normals and vertex positions
 */
# define MAX_LIGHTS 8
uniform int num_lights;
varying vec3 normal;
varying vec3 viewPos;
varying vec3 lightDir[MAX_LIGHTS];

void main()
{
    normal = normalize(gl_NormalMatrix * gl_Normal);
    viewPos = vec3(gl_ModelViewMatrix * gl_Vertex);
    for (int i = 0; i < num_lights; i++) {
        lightDir[i] = vec3(gl_LightSource[i].position)-viewPos;
    }
    gl_Position = ftransform();
}