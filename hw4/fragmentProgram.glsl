/* Fragment shader for the Phong shading implementation. 
 * Automatically interpolates normals and vertex positions. 
 * Sets the output color of the pixel to the
 */
#define MAX_LIGHTS 8
uniform int num_lights;
varying vec3 normal;
varying vec3 viewPos;
varying vec3 lightDir[MAX_LIGHTS];

void main()
{
    vec3 c_d = vec3(gl_FrontMaterial.diffuse);
    vec3 c_a = vec3(gl_FrontMaterial.ambient);
    vec3 c_s = vec3(gl_FrontMaterial.specular);
    float p = gl_FrontMaterial.shininess;
    vec3 color = c_a;
    vec3 e_dir = normalize(-viewPos);
    for (int i = 0; i < num_lights; i++) 
    {   
        float dist = length(lightDir[i]);
        float factor = 1.0 / (1.0 + gl_LightSource[i].quadraticAttenuation * 
                        pow(dist, 2.0));
        vec3 normDir = normalize(lightDir[i]);
        vec3 l_diffuse = factor * vec3(gl_LightSource[i].diffuse) * 
                        max(dot(normal, normDir), 0.0);
        vec3 l_specular = factor * vec3(gl_LightSource[i].specular) * 
                    max(pow(dot(normal, normalize(normDir + e_dir)), p), 0.0);
        l_diffuse *= c_d;
        l_specular *= c_s;
        color += l_diffuse + l_specular;
    }
    gl_FragColor = clamp(vec4(color, 1.0), 0.0, 1.0);
}
