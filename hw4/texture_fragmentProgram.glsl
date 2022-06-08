/* Implements flat shading using ambient and diffuse lighting. Takes in material
 * color and surface normal from texture maps.
 */
 
#define MAX_LIGHTS 8
uniform sampler2D texture, heightmap;
varying vec3 lightDir[MAX_LIGHTS];

void main()
{  
    vec3 normal = texture2D(heightmap, gl_TexCoord[0].st).rgb;
    vec3 texColor = vec3(texture2D(texture, gl_TexCoord[0].st));
    normal = normalize(2.0 * normal - 1.0);
    vec3 color = vec3(0.0);
    for (int i = 0; i < MAX_LIGHTS; i++) {
        vec3 out_color = texColor * vec3(gl_LightSource[i].ambient);
        vec3 normDir = normalize(lightDir[i]);
        float n_dot_l = dot(normal, normDir);
        if (n_dot_l > 0.0) {
            out_color += texColor * vec3(gl_LightSource[i].diffuse) * n_dot_l;
        }
        color += out_color;
    }
    gl_FragColor = clamp(vec4(color, 1.0), 0.0, 1.0);
}
