#include "Assignment.hpp"

#include "model.hpp"
#include "UI.hpp"
#include "Scene.hpp"
#include "PNGMaker.hpp"

#define XRES 250
#define YRES 250

using namespace std;

namespace Assignment {
    /* Assignment Part A, Part 1 */
    static const float k_min_io_test_x = -7.0;
    static const float k_max_io_test_x = 7.0;
    static const float k_min_io_test_y = -7.0;
    static const float k_max_io_test_y = 7.0;
    static const float k_min_io_test_z = -7.0;
    static const float k_max_io_test_z = 7.0;
    static const int k_io_test_x_count = 15;
    static const int k_io_test_y_count = 15;
    static const int k_io_test_z_count = 15;

    /* Computes the rotation matrix given the direction of the axis and rotation
	 * angle. Importantly, normalizes the axis direction before computing the
	 * matrix.
	 */
	Matrix4f getRotationMatrix(float x, float y, float z, float theta) {
	    Vector3f vec;
	    vec << x, y, z;
	    vec.normalize();
	    x = vec[0];
	    y = vec[1];
	    z = vec[2];
	    float A = pow(x, 2) + (1 - pow(x, 2)) * cos(theta);
	    float B = x * y * (1 - cos(theta)) - z * sin(theta);
	    float C = x * z * (1 - cos(theta)) + y * sin(theta);
	    float D = x * y * (1 - cos(theta)) + z * sin(theta);
	    float E = pow(y, 2) + (1 - pow(y, 2)) * cos(theta);
	    float F = y * z * (1 - cos(theta)) - x * sin(theta);
	    float G = x * z * (1 - cos(theta)) - y * sin(theta);
	    float H = y * z * (1 - cos(theta)) + x * sin(theta);
	    float I = pow(z, 2) + (1 - pow(z, 2)) * cos(theta);
	    Matrix4f m;
	    m << A, B, C, 0,   
	        D, E, F, 0,    
	        G, H, I, 0,  
	        0, 0, 0, 1;
	    return m;
	}

	/* Returns the value of the general inside-out function at (x, y, z) given
	 * the superquadric's coefficients, as well as e and n.
	 */
	float getVal(Vector3f coeff, float x, float y, float z, float e, float n) {
		 float val = pow(pow(pow(x / coeff[0], 2.0), 1.0 / e) + 
            	 pow(pow(y / coeff[1], 2.0), 1.0 / e), e / n) +
            	 pow(pow(z / coeff[2], 2.0), 1.0 / n) - 1.0;
		 return val;
	}

	/* Returns the overall transformation matrix on a superquadric. 
	 * The boolean value scale is true if we include translation
	 * transformations, false if not
	 */
	Matrix4f getTransformMat(vector<Transformation> &transformation_stack, 
							bool translate) {
		Matrix4f mult = Matrix4f::Identity();
        for (Transformation tr : transformation_stack) {
        	Matrix4f m = Matrix4f::Identity();
        	if (tr.type == TRANS && translate) {
        		m << 1, 0, 0, tr.trans[0], 
        			 0, 1, 0, tr.trans[1],
        			 0, 0, 1, tr.trans[2],
        			 0, 0, 0, 1;
        	}
        	else if (tr.type == SCALE) {
        		m << tr.trans[0], 0, 0, 0, 
        			 0, tr.trans[1], 0, 0,
        			 0, 0, tr.trans[2], 0,
        			 0, 0, 0, 1;
        	}
        	else if (tr.type == ROTATE) {
        		m << getRotationMatrix(tr.trans[0], tr.trans[1],
        			tr.trans[2], tr.trans[3]);
        	}
        	mult = mult * m;
        }
        return mult;
	}

    bool IOTest(
        Renderable *ren,
        vector<Transformation> &transformation_stack,
        float x,
        float y,
        float z)
    {
        if (ren->getType() == PRM) {
            Primitive *prm = dynamic_cast<Primitive*>(ren);

            /* TODO
             *
             *     Apply inverse of the transformations passed in through
             * transformation_stack to (x, y, z) and perform the IO test
             * oulined in the lecture notes. If the point is inside the
             * primitive, return true. Otherwise return false
             **/
            Vector4f point;
            point << x, y, z, 1;
            Vector4f trans = getTransformMat(transformation_stack, 
                            true).inverse() * point;
            float e = prm->getExp0();
            float n = prm->getExp1();
            float new_x = trans[0];
            float new_y = trans[1];
            float new_z = trans[2];
            Vector3f coeff = prm->getCoeff();
            float val = getVal(coeff, new_x, new_y, new_z, e, n);

            /* If the inside-out function evalutes to > 0, 
            the point is outside the object */
            if (val > 0.0) {
            	return false;
            }
            return true; 

        } else if (ren->getType() == OBJ) {
            Object *obj = dynamic_cast<Object*>(ren);
            const vector<Transformation>& overall_trans =
                obj->getOverallTransformation();
            for (int i = overall_trans.size() - 1; i >= 0; i--) {
                transformation_stack.push_back(overall_trans.at(i));
            }

            bool IO_result = false;
            for (auto& child_it : obj->getChildren()) {
                const vector<Transformation>& child_trans = 
                    child_it.second.transformations;
                for (int i = child_trans.size() - 1; i >= 0; i--) {
                    transformation_stack.push_back(child_trans.at(i));
                }
                IO_result |= IOTest(
                    Renderable::get(child_it.second.name),
                    transformation_stack,
                    x, y, z);
                transformation_stack.erase(
                    transformation_stack.end() - child_trans.size(),
                    transformation_stack.end());
            }

            transformation_stack.erase(
                transformation_stack.end() - overall_trans.size(),
                transformation_stack.end());
            return IO_result;
        } else {
            fprintf(stderr, "Renderer::draw ERROR invalid RenderableType %d\n",
                ren->getType());
            exit(1);
        }

        return true;
    }

    void drawIOTest() {
        const Line* cur_state = CommandLine::getState();
        Renderable* current_selected_ren = NULL;

        if (cur_state) {
            current_selected_ren = Renderable::get(cur_state->tokens[1]);
        }

        if (current_selected_ren == NULL) {
            return;
        }

        const float IO_test_color[3] = {0.5, 0.0, 0.0};
        glMaterialfv(GL_FRONT, GL_AMBIENT, IO_test_color);
        for (int x = 0; x < k_io_test_x_count; x++) {
            for (int y = 0; y < k_io_test_y_count; y++) {
                for (int z = 0; z < k_io_test_z_count; z++) {
                    float test_x = k_min_io_test_x
                        + x * (k_max_io_test_x - k_min_io_test_x)
                            / (float) k_io_test_x_count;
                    float test_y = k_min_io_test_y
                        + y * (k_max_io_test_y - k_min_io_test_y)
                            / (float) k_io_test_y_count;
                    float test_z = k_min_io_test_z
                        + z * (k_max_io_test_z - k_min_io_test_z)
                            / (float) k_io_test_z_count;

                    vector<Transformation> transformation_stack;
                    if (IOTest(
                            current_selected_ren,
                            transformation_stack,
                            test_x,
                            test_y,
                            test_z))
                    {
                        glPushMatrix();
                        glTranslatef(test_x, test_y, test_z);
                        glutWireSphere(0.05, 4, 4);
                        glPopMatrix();
                    }
                }
            }
        }
    }

    /* Assignment Part A, Part 2 */
    struct Ray {
        float origin_x;
        float origin_y;
        float origin_z;

        float direction_x;
        float direction_y;
        float direction_z;

        Vector3f getLocation(float t) {
            Vector3f loc;
            loc << origin_x + t * direction_x,
                origin_y + t * direction_y,
                origin_z + t * direction_z;
            return loc;
        }
    };

    /* A struct for containing both the overall transformations on a primitive
     * as well as the pointer to the actual primitive 
     */
    struct PrimTransform {
    	vector<Transformation> tr;
    	Primitive* prim;
    };

    /* A struct for containing both the result, which is a float, and whether
     * or not a result was actually found. The default constructor initializes
     * a struct where the found boolean is set to false, so this struct is
     * returned if a result was not found.
     */
    struct FloatResult {
    	float res;
    	bool found;
    	FloatResult() : res(0), found(false) {};
    };

    /* A struct for containing both the result, which is a pair, and whether
     * or not a result was actually found. The default constructor initializes
     * a struct where the found boolean is set to false, so this struct is
     * returned if a result was not found.
     */
    struct PairResult {
        pair<float, float> res;
        bool found;
        PairResult() : res(pair<float, float>(0, 0)), found(false) {};
    };


    /* Takes in a current renderable object and modifies the out vector
     * to contain all the primitives in the scene. The transformation_stack
     * vector contains the transformations to the current renderable
     */
    void getPrimitivesRec(Renderable* ren, vector<PrimTransform>& out, 
    					vector<Transformation>& transformation_stack) {
        if (ren->getType() == PRM) {
            Primitive *prm = dynamic_cast<Primitive*>(ren);
            PrimTransform pt; 
            pt.tr = transformation_stack;
            pt.prim = prm;
            out.push_back(pt);
        }
        else if (ren->getType() == OBJ) {
            Object *obj = dynamic_cast<Object*>(ren);
            const vector<Transformation>& overall_trans =
                obj->getOverallTransformation();
            for (int i = overall_trans.size() - 1; i >= 0; i--) {
                transformation_stack.push_back(overall_trans.at(i));
            }

            for (auto& child_it : obj->getChildren()) {
                const vector<Transformation>& child_trans = 
                    child_it.second.transformations;
                for (int i = child_trans.size() - 1; i >= 0; i--) {
                    transformation_stack.push_back(child_trans.at(i));
                }
                getPrimitivesRec(Renderable::get(child_it.second.name), out, 
                								transformation_stack);
                transformation_stack.erase(
                    transformation_stack.end() - child_trans.size(),
                    transformation_stack.end());
            }
            transformation_stack.erase(
                transformation_stack.end() - overall_trans.size(),
                transformation_stack.end());
        }
    } 

    /* If we are not provided the scene, use the command line to get 
     * the current Renderable, and then use recursive to get any children
     * superquadrics
     */
    vector<PrimTransform> getAllPrimitives() {
        // Iterate through all objects in the scene
        const Line* cur_state = CommandLine::getState();
        Renderable* current_selected_ren = NULL;

        if (cur_state) {
            current_selected_ren = Renderable::get(cur_state->tokens[1]);
        }
        vector<PrimTransform> prm;
        if (current_selected_ren == NULL) {
            return prm;
        }
        vector<Transformation> tr_stack;
        getPrimitivesRec(current_selected_ren, prm, tr_stack);
        return prm;
    }

    /* Iterate over the passed in scene's root objects and then use recursion
     * to get any non-root superquadrics
     */
    vector<PrimTransform> getPrimitivesFromScene(Scene scene) {
        // Iterate through all objects in the scene
        vector<PrimTransform> prm;
        vector<Transformation> tr_stack;
        for (Object *obj : scene.root_objs) {
            getPrimitivesRec(obj, prm, tr_stack);
        }
        return prm;
    }

    /* Return the two possible initial guesses t+ and t- as the
     * potential starting points for Newton's method
     */
    PairResult getStartingT(Vector3f dir, Vector3f pos, PrimTransform pt) {
        Vector4f new_pos;
        new_pos << pos, 1;
        Vector4f new_dir;
        new_dir << dir, 1;
        Vector3f tr_pos;

        // Transform the passed in ray direction and origin by the inverse
        // transformation matrix of the superquadric
        tr_pos << (getTransformMat(pt.tr, true).inverse() * new_pos).head(3);
        Vector3f tr_dir;
        tr_dir << (getTransformMat(pt.tr, false).inverse() * new_dir).head(3);

    	float a = tr_dir.dot(tr_dir);
    	float b = 2.0 * tr_pos.dot(tr_dir);

        // Take into account the superquadric's own scaling
        float coeff = (pt.prim->getCoeff()).maxCoeff();
        float radsq = 3 * pow(coeff, 2);
        float c = tr_pos.dot(tr_pos) - radsq;
    	float disc = pow(b, 2) - 4.0 * a * c;

        PairResult pr;
    	// If the discriminant is 0, the ray misses the 
    	// bounding sphere entirely, so there is no solution
    	if (disc < 0) {
    		return pr;
    	}
    	float t_min = (-b - sqrt(disc)) / (2.0 * a);
    	float t_pos = (-b + sqrt(disc)) / (2.0 * a);
        pr.res = pair<float, float>(t_min, t_pos);
        pr.found = true;
    	return pr;
    }

    /* Computes the gradient of a superquadric function at (x, y, z), given
     * the superquadric's coefficients and its exponents
     */
    Vector3f getGradient(Vector3f coeff, float x, float y, float z, float e, 
                            float n) {
        float new_x = x / coeff[0];
        float new_y = y / coeff[1];
        float new_z = z / coeff[2];
        Vector3f grad;

        grad(0) = (new_x == 0.0) ? 0.0 :
            2.0 * new_x * powf(new_x * new_x, 1.0 / e - 1.0) *
            powf(powf(new_x * new_x, 1.0 / e) + powf(new_y * new_y, 1.0 / e),
                e / n - 1.0) / n;
        grad(1) = (new_y == 0.0) ? 0.0 :
            2.0 * new_y * powf(new_y * new_y, 1.0 / e - 1.0) *
            powf(powf(new_x * new_x, 1.0 / e) + powf(new_y * new_y, 1.0 / e),
                e / n - 1.0) / n;
        grad(2) = (new_z == 0.0) ? 0.0 :
            2.0 * new_z * powf(new_z * new_z, 1.0 / n - 1.0) / n;

        grad[0] /= coeff[0];
        grad[1] /= coeff[1];
        grad[2] /= coeff[2];
    	return grad;
    }

    /* Apply the Newton Method to find the time T at which a ray, defined by
     * its direction and position, intersects the primitive. If there is no
     * intersection, return a FloatResult with the boolean 'found' value set
     * to false 
     */
    FloatResult newtonMethod(Vector3f dir, Vector3f pos, float startT, 
                            PrimTransform pt) {
    	float oldT = startT;
    	// Initialize these values to something that's not 0
    	float currVal = 1;
    	float deriv = -1;
    	float bound = 1e-3;
    	Primitive *prm = pt.prim;
    	while (deriv < 0 && abs(deriv) > bound && abs(currVal) > bound) {
    		Vector3f currVec = dir * oldT + pos;
    		// Apply inverse transformations to the ray
    		Vector4f vec;
    		vec << currVec, 1;
    		Vector4f trans = getTransformMat(pt.tr, true).inverse() * vec;

    		// Compute g(t)
	    	currVal = getVal(prm->getCoeff(), trans[0], trans[1],
	    				trans[2], prm->getExp0(), prm->getExp1());
	    	Vector3f grad = getGradient(prm->getCoeff(), trans[0], trans[1],
	    				trans[2], prm->getExp0(), prm->getExp1());

            // Apply the appropriate inverse transformation to the surface
            // normal
            Vector4f normal;
            normal << grad, 1;
            Matrix4f mat = getTransformMat(pt.tr, false).inverse().transpose();
            Vector4f trans_norm = mat * normal;
            Vector3f norm = trans_norm.head(3);

	    	// Compute g'(t) using the gradient of the superquadric function
	    	deriv = dir.dot(norm);
	    	if (abs(deriv) > bound) {
	    		oldT = oldT - currVal / deriv;
	    	}
    	}
    	FloatResult fr;
    	if (abs(currVal) < bound) {
    		fr.res = oldT;
    		fr.found = true;
    		return fr;
    	}
    	return fr;
    }	

    /* Finds the intersection of the passed in ray with the nearest primitive.
     * If an intersection is found, the foundPrim pointer is modified with the 
     * pointer to the intersected primitive. Reads in primitives from the scene
     * if the scene is given
     */
    Ray findIntersection(const Ray &camera_ray, Primitive*& foundPrim, 
                            Scene *scene) {
        /* TODO
         *
         **/
        Ray intersection_ray;
        intersection_ray.origin_x = 0.0;
        intersection_ray.origin_y = 0.0;
        intersection_ray.origin_z = 0.0;
        intersection_ray.direction_x = 0.0;
        intersection_ray.direction_y = 0.0;
        intersection_ray.direction_z = 0.0;

        Vector3f pos;
        pos << camera_ray.origin_x, camera_ray.origin_y, 
        				camera_ray.origin_z;
        Vector3f dir;
        dir << camera_ray.direction_x, 
        			camera_ray.direction_y, camera_ray.direction_z;

        vector<PrimTransform> prm;
        if (scene == NULL) {
            prm = getAllPrimitives(); 
        }
        else {
            prm = getPrimitivesFromScene(*scene);  
        }  
        if (prm.size() == 0) {
            return intersection_ray;
        }   
        // Now go through all the primitives and find the closest
        // intersection
        PrimTransform closestPrim;
        // Store the intersection point
        Vector3f closest;
        // Stores the distance between the intersection point
        // and the camera
        float dist = -1;
        for (PrimTransform pt : prm) {

    		PairResult pr = getStartingT(dir, pos, pt);
            if (! pr.found) {
                continue;
            }
            pair<float, float> startT = pr.res;
    		FloatResult fr;
    		if (startT.first > 0 && startT.second > 0) {
    			fr = newtonMethod(dir, pos, startT.first, pt);
    		}
    		else if (startT.first < 0 && startT.second > 0) {
    			FloatResult fr1 = newtonMethod(dir, pos, startT.first, pt);

                // We only need to consider the result of the smaller t-value
                // since if the solution is positive, the camera is not in the 
                // superquadric or in front of it
                if (fr1.found == true && fr1.res > 0) {
    				fr = fr1;
    			}
    			else {
                    // If one of the solutions is negative then the camera is  
                    // inside the superquadric
    				continue;
    			}
    		}
    		else {
    			// If both start values of t are negative, then no 
    			// solution exists
    			continue;
    		}
    		if (fr.found == true) {
    			Vector3f point = dir * fr.res + pos;
        		float newDist = (point - pos).norm(); 
        		if (dist < 0 || newDist < dist) {
        			dist = newDist;
        			closest << point;
        			closestPrim = pt;
        		}
    		}
        		
        }
        if (dist < 0) {
        	return intersection_ray;
        }
        // Set the origin to the point on the surface of the superquadric
        intersection_ray.origin_x = closest[0];
        intersection_ray.origin_y = closest[1];
        intersection_ray.origin_z = closest[2];

        // Apply the inverse transformation to the closest point and 
        // call the superquadric's getNormal
        Vector4f vec;
        vec << closest, 1;
        Vector4f trans = getTransformMat(closestPrim.tr, true).inverse() * vec;
        Vector4f normal;
        normal << closestPrim.prim->getNormal(trans.head(3)), 1;
        Matrix4f mat = getTransformMat(closestPrim.tr, 
                        false).inverse().transpose();
        Vector4f trans_normal = mat * normal;
        Vector3f scaled_normal = trans_normal.head(3);
        scaled_normal.normalize();
        foundPrim = closestPrim.prim;

        intersection_ray.direction_x = scaled_normal[0];
        intersection_ray.direction_y = scaled_normal[1];
        intersection_ray.direction_z = scaled_normal[2];
        return intersection_ray;
    }

    /* Transforms the camera's direction and position by its rotation matrix and
     * returns the new direction and position
     */
    pair<Vector3f, Vector3f> getCameraDirPos(Camera *camera) {
        // Transform the camera ray direction
        Vector3f vec = camera->getAxis();
        float angle = camera->getAngle();
        Vector4f new_dir; 
        new_dir << 0.0f, 0.0f, -1.0f, 1.0f;
        Matrix4f mat = getRotationMatrix(vec[0], vec[1], vec[2], 
                            angle);
        new_dir = mat * new_dir;
        Vector3f dir = new_dir.head(3);
        dir.normalize();

        // Also transform the camera's position
        Vector3f pos = camera->getPosition();
        Vector4f new_pos; 
        new_pos << pos[0], pos[1], pos[2], 1.0f;
        new_pos = mat * new_pos;
        pos = new_pos.head(3);
        return pair<Vector3f, Vector3f>(dir, pos);
    }

    void drawIntersectTest(Camera *camera) {
        Ray camera_ray;
        pair<Vector3f, Vector3f> cam = getCameraDirPos(camera);
        Vector3f dir = cam.first;
        Vector3f pos = cam.second;

        camera_ray.origin_x = pos[0];      
        camera_ray.origin_y = pos[1];      
        camera_ray.origin_z = pos[2];     
        camera_ray.direction_x = dir[0];   
        camera_ray.direction_y = dir[1];   
        camera_ray.direction_z = dir[2];  
        Primitive* foundPrim = NULL; 
        Ray intersection_ray = findIntersection(camera_ray, foundPrim, NULL);
        const float IO_test_color[3] = {1.0, 0.0, 0.5};
        glMaterialfv(GL_FRONT, GL_AMBIENT, IO_test_color);
        glLineWidth(3.0);
        glBegin(GL_LINES);
        glVertex3f(
            intersection_ray.origin_x,
            intersection_ray.origin_y,
            intersection_ray.origin_z);
        Vector3f endpoint = intersection_ray.getLocation(1.0);
        glVertex3f(endpoint[0], endpoint[1], endpoint[2]);
        glEnd();
    }

    /* Assignment Part B */

    /* Compute the width and height of the screen plane using the frustum's near
     * component and the camera's field of view and aspect ratio
     */
    pair<float, float> getWidthHeight(Camera camera) {
        float n = camera.near;
        float theta = degToRad(camera.fov);
        float h = 2 * n * tan(theta / 2);
        float w = camera.aspect * h;
        return pair<float, float>(w, h);
    }

    /* Divide the screen plane into a grid and returns a ray corresponding
     * to the light ray from the camera to pixel (i, j). Takes in the 
     * width and height of the plane, and the near component of the frustum
     */
    Ray getRayThroughPoint(float width, float height, int i, int j, 
                            float n, Vec3f pos) {
        Ray point_ray;
        point_ray.origin_x = pos.x;
        point_ray.origin_y = pos.y;
        point_ray.origin_z = pos.z;
        Vector3f e1;
        e1 << 0.0f, 0.0f, -1.0f;
        Vector3f e2;
        e2 << 1.0f, 0.0f, 0.0f;
        Vector3f e3;
        e3 << 0.0f, 1.0f, 0.0f;
        Vector3f dir;
        float x = (float) XRES;
        float y = (float) YRES;
 
        float xi = (i - (x / 2.0)) * (width / x);
        float yj = (j - (y / 2.0)) * (height / y);
        dir = n * e1 + xi * e2 + yj * e3;
        point_ray.direction_x = dir[0];
        point_ray.direction_y = dir[1];
        point_ray.direction_z = dir[2];
        return point_ray;
    }

    /* Check if a light intersects with a primitive in the sceneother
     * than the passed in primitive.
     */
    bool findLightIntersection(Vector3f l_pos, Vector3f l_dir, 
                            Primitive *currPrim, Scene scene) {
        Ray light_ray;
        light_ray.origin_x = l_pos[0];
        light_ray.origin_y = l_pos[1];
        light_ray.origin_z = l_pos[2];
        light_ray.direction_x = l_dir[0];
        light_ray.direction_y = l_dir[1];
        light_ray.direction_z = l_dir[2];

        Primitive* foundPrim = NULL;
        findIntersection(light_ray, foundPrim, &scene);

        // The light is obstructed by a different object than the one 
        // we're currently rendering 
        if (foundPrim != currPrim && foundPrim != NULL) {
            return true;
        }
        return false;
    }

    /* Takes in the surface normal ray for a point on the superquadric.
     * Also takes in the camera's position and the scene. Iterates over 
     * the scene's 
     */
    Vector3f phongShading(Ray normal, Primitive *prim, Vector3f cam_pos,
                        Scene scene) {
        Vector3f pos = normal.getLocation(0);
        vector<PointLight> lights = scene.lights;

        RGBf col = prim->getColor();
        Vector3f color;
        color << col.r, col.g, col.b;
        float diff = prim->getDiffuse();
        float amb = prim->getAmbient();
        float spec = prim->getSpecular();
        Vector3f c_d = diff * color;
        Vector3f c_a = amb * color;
        Vector3f c_s = spec * color;
        Vector3f diffuse_sum = Vector3f::Zero();
        Vector3f specular_sum = Vector3f::Zero();
        Vector3f e_dir = cam_pos - pos; 
        Vector3f n;
        n << normal.direction_x, normal.direction_y, normal.direction_z;
        n.normalize();
        e_dir.normalize();
        for (PointLight l : lights) {
            Vector3f l_pos; 
            l_pos << l.position[0], l.position[1], l.position[2]; 
            Vector3f l_dir = l_pos - pos;

            float dist = l_dir.norm();
            l_dir.normalize();
            // Check if there are any occluding objects that block 
            // the light ray to the current object
            if (findLightIntersection(l_pos, -l_dir, prim, scene)) {
                continue;
            }
            Vector3f l_c;
            l_c << l.color[0], l.color[1], l.color[2];

            // Incorporate attenuation
            float factor = 1.0f / (1.0f + l.k * pow(dist, 2));
            l_c *= factor;

            Vector3f l_diffuse = l_c * max(0.0f, n.dot(l_dir));
            diffuse_sum += l_diffuse;
            Vector3f sum_dir = e_dir + l_dir;
            sum_dir.normalize();
            float shiny = prim->getReflected();
            Vector3f l_specular = l_c * pow(max(0.0f, n.dot(sum_dir)), 
                                shiny);
            specular_sum += l_specular;
        }
        Vector3f allOnes = Vector3f::Ones();
        Vector3f calc = c_a + diffuse_sum.cwiseProduct(c_d) + 
                            specular_sum.cwiseProduct(c_s); 

        // Color values can't exceed 1
        Vector3f result = allOnes.cwiseMin(calc);
        return result;
    }

    /* Ray traces the scene. */
    void raytrace(Camera camera, Scene scene) {
        // LEAVE THIS UNLESS YOU WANT TO WRITE YOUR OWN OUTPUT FUNCTION
        PNGMaker png = PNGMaker(XRES, YRES);

        pair<float, float> wh = getWidthHeight(camera);
        // Transform the camera position
        Vector3f pos = camera.getPosition();
        Vector4f new_pos; 
        new_pos << pos[0], pos[1], pos[2], 1.0f;
        Vector3f vec = camera.getAxis();
        float angle = camera.getAngle();
        Matrix4f mat = getRotationMatrix(vec[0], vec[1], vec[2], 
                                    angle);
        new_pos = mat * new_pos;
        pos = new_pos.head(3);
        for (int i = 0; i < XRES; i++) {
            for (int j = 0; j < YRES; j++) {
                // Print out the current pixel to keep track of the raytracing
                // progress
                cout << i << " " << j << endl;
                Ray point_ray = getRayThroughPoint(wh.first, wh.second, i, j, 
                            camera.near, camera.position);
                // Transform the point ray by the camera rotation
                Vector4f curr_dir;
                curr_dir << point_ray.direction_x, point_ray.direction_y, 
                            point_ray.direction_z, 1.0f;
                Vector4f new_dir = mat * curr_dir;
                Vector3f dir = new_dir.head(3);

                point_ray.direction_x = dir[0];
                point_ray.direction_y = dir[1];
                point_ray.direction_z = dir[2];
                point_ray.origin_x = pos[0];
                point_ray.origin_y = pos[1];
                point_ray.origin_z = pos[2];
                Primitive* foundPrim = NULL;
                Ray normal = findIntersection(point_ray, foundPrim, &scene); 
                if (foundPrim != NULL) {
                    Vector3f color = phongShading(normal, foundPrim, 
                        pos, scene);
                    png.setPixel(i, j, color[0], color[1], color[2]);
                }
            }
        }

        // LEAVE THIS UNLESS YOU WANT TO WRITE YOUR OWN OUTPUT FUNCTION
        if (png.saveImage()) {
            fprintf(stderr, "Error: couldn't save PNG image\n");
        } else {
            printf("DONE!\n");
        }
    }
};
