// car.cpp

#include "CarBot.h"
using namespace Unigine;
using namespace Math;
using namespace std;
/// function setting up bodies for a wheel joint
void setBodies(JointWheelPtr joint, const BodyRigidPtr &car_body, const BodyRigidPtr &wheel_body)
{
	joint->setBody0(car_body);
	joint->setBody1(wheel_body);

	Mat4 wheel_t = wheel_body->getObject()->getTransform();

	Vec3 anchor0 = wheel_t.getTranslate();
	Vec3 anchor1 = Vec3_zero;

	vec3 axis00 = vec3_up;
	vec3 axis10 = vec3_right;
	vec3 axis11 = static_cast<vec3>(rotation(wheel_t) * axis10);

	// setting up joint's anchor and axes
	joint->setAnchor0(anchor0);
	joint->setAnchor1(anchor1);

	joint->setAxis00(axis00);
	joint->setAxis10(axis10);
	joint->setAxis11(axis11);
}

/// function, creating a car body having a specified size, color and transformation with a body and a shape
ObjectMeshDynamicPtr createBody(char *name, const vec3& size, float mass, const vec4& color, const Mat4& transform)
{
	// creating a car body and setting up its parameters (name, material and transformation)
	ObjectMeshDynamicPtr OMD = Primitives::createBox(size);

	OMD->setWorldTransform(transform);
	OMD->setMaterial("mesh_base", "*");
	OMD->setMaterialParameterFloat4("albedo_color", color, 0);
	OMD->setName(name);

	// enabling collision detection for the frame
	OMD->setCollision(1, 0);

	// adding physics, i.e. a rigid body and a box shape with specified mass
	BodyRigidPtr body = BodyRigid::create(OMD);

	body->addShape(ShapeBox::create(size), translate(vec3(0.0f)));
	OMD->getBody()->getShape(0)->setMass(mass);

	return OMD;
}

/// function, creating a wheel having a specified size and transformation
/// with a physical body and attaching it to the frame
ObjectMeshDynamicPtr createWheel(NodePtr frame, char *name, float radius, float height, const Mat4& transform)
{
	// creating a wheel, adding it to the frame as a child,
	// and setting up its parameters (name, material and transformation)
	ObjectMeshDynamicPtr OMD = Primitives::createCylinder(radius, height, 1, 32);
	frame->addChild(OMD);

	OMD->setTransform(transform);
	OMD->setMaterial("mesh_base", "*");
	OMD->setMaterialParameterFloat4("albedo_color", vec4_black, 0);
	OMD->setName(name);

	// enabling collision detection for the wheel
	OMD->setCollision(1, 0);

	// adding a rigid body 
	BodyRigidPtr body = BodyRigid::create(OMD);

	return OMD;
}

/// Initializing a car with specified frame and wheel parameters
int CarBot::init(int num,vec3 position,float blength, float bwidth, float bheight, float wradius, float wwidth, float wmass, const Mat4& transform)
{
    viewport = Viewport::create();

    car_num = num;
    time = t.getTime();
    im_count = 0;

	frame_width = bwidth;
	frame_height = bheight;
	frame_length = blength;
	wheel_radius = wradius;
	wheel_width = wwidth;

	float delta = 0.2f;

	car_frame = createBody("car_frame", vec3(frame_width, frame_length, frame_height), 64.0f, vec4::RED, transform);

	// initialization of wheels 
	wheels[0] = createWheel(car_frame, "car_wheel_f_l", wheel_radius, wheel_width, Mat4(translate(-(frame_width + wheel_width) / 2 - delta, frame_length / 2 - wheel_radius, -frame_height / 2)* rotateY(90.0f)));
	wheels[1] = createWheel(car_frame, "car_wheel_f_r", wheel_radius, wheel_width, Mat4(translate((frame_width + wheel_width) / 2 + delta, frame_length / 2 - wheel_radius, -frame_height / 2)* rotateY(90.0f)));

	wheels[2] = createWheel(car_frame, "car_wheel_r_l", wheel_radius, wheel_width, Mat4(translate(-(frame_width + wheel_width) / 2 - delta, -0.25f * frame_length, -frame_height / 2) * rotateY(90.0f)));
	wheels[3] = createWheel(car_frame, "car_wheel_r_r", wheel_radius, wheel_width, Mat4(translate((frame_width + wheel_width) / 2 + delta, -0.25f * frame_length, -frame_height / 2) * rotateY(90.0f)));

	// initialization of wheel joints
	for (int i = 0; i < 4; i++)
	{
		wheel_joints[i] = JointWheel::create();

		// setting bodies and wheel parameters
		setBodies(wheel_joints[i], car_frame->getBodyRigid(), wheels[i]->getBodyRigid());
		wheel_joints[i]->setWheelRadius(wradius);
		wheel_joints[i]->setWheelMass(wmass);

		// setting restitution parameters
		wheel_joints[i]->setLinearRestitution(0.1f);
		wheel_joints[i]->setAngularRestitution(0.1f);

		// setting linear damping and spring rigidity
		wheel_joints[i]->setLinearDamping(400.0f);
		wheel_joints[i]->setLinearSpring(100.0f);

		// setting lower and upper suspension ride limits [-0.15; 0.15]
		wheel_joints[i]->setLinearLimitFrom(-0.15f);
		wheel_joints[i]->setLinearLimitTo(0.15f);
		wheel_joints[i]->setLinearDistance(0.0f);
		// setting number of iterations
		wheel_joints[i]->setNumIterations(8);
	}

	// setting up player and controls
	player = PlayerPersecutor::create();

	player->setFixed(1);
	player->setTarget(car_frame);
	player->setMinDistance(6.0f);
	player->setMaxDistance(11.0f);
	player->setPosition(Vec3(0.0f, -10.0f, 6.0f));
	player->setControlled(0);
	Game::setPlayer(player);
	Game::setEnabled(1);
    
    player->setTarget(car_frame);
    car_frame->setPosition(position);

    direction[0] = 1;
    direction[1] = 0;
    direction[2] = 1;
    direction[3] = 0;

	return 1;
}

/// method updating current car state with a keyboard control handler
void CarBot::Move()
{
    float ifps = Game::getIFps();

    // forward and backward movement by setting joint motor's velocity and torque
    if (direction[0])
    {
        velocity = max(velocity, 0.0f);
        velocity += ifps * 50.0f;
        torque = 5.0f;
    }
    else if (direction[1])
    {
        velocity = min(velocity, 0.0f);
        velocity -= ifps * 50.0f;
        torque = 5.0f;
    }
    else
    {
        velocity *= Math::exp(-ifps);
    }
    velocity = clamp(velocity, -90.0f, 90.0f);

    // steering left and right by changing Axis01 for front wheel joints
    if (direction[2])
        angle += ifps * 100.0f;
    else if (direction[3])
        angle -= ifps * 100.0f;
    else
    {
        if (Math::abs(angle) < 0.25f) angle = 0.0f;
        else angle -= sign(angle) * ifps * 45.0f;
    }
    angle = clamp(angle, -30.0f, 30.0f);

    // calculating steering angles for front joints (angle_0 and angle_1)
    float base = 3.3f;
    float width = 3.0f;
    float angle_0 = angle;
    float angle_1 = angle;
    if (Math::abs(angle) > Consts::EPS)
    {
        float radius = base / Math::tan(angle * Consts::DEG2RAD);
        angle_0 = Math::atan(base / (radius + width / 2.0f)) * Consts::RAD2DEG;
        angle_1 = Math::atan(base / (radius - width / 2.0f)) * Consts::RAD2DEG;
    }

    wheel_joints[0]->setAxis10(rotateZ(angle_0).getColumn3(0));
    wheel_joints[1]->setAxis10(rotateZ(angle_1).getColumn3(0));
}
int CarBot::update()
{
    
    if (t.getTime() - time > time_change)
    {
        time = t.getTime();
        if (!screenshot)
        {
        screenshot = Texture::create();
        }
        screenshot->create2D(30, 30, Texture::FORMAT_RGBA8, Texture::USAGE_RENDER); // create 30 x 30 render target

        // render 30x30 Image
        rendertarget = RenderTarget::create();
        rendertarget->bindColorTexture(0, screenshot);
        rendertarget->enable();
        viewport->render(player->getCamera());
        rendertarget->disable();
        rendertarget->unbindAll();
        ImagePtr screenshot_image = Image::create();
        screenshot->getImage(screenshot_image);
        screenshot_image->convertToFormat(Image::FORMAT_RGB8);
        screenshot_image->save(("scr" + to_string(car_num)+to_string(im_count) + ".png").c_str());
        
        // get r,g,b pixels
        int N = screenshot_image->getHeight();
        int M = screenshot_image->getWidth();
        Image::Pixel px;
        for (int i = 0; i < M; i++)
        {
            for (int j = 0; j < N; j++)
            {
                px = screenshot_image->get2D(i, j);
                red_mask[i][j] = px.i.r;
                green_mask[i][j] = px.i.g;
                blue_mask[i][j] = px.i.b;
            }
        }
    }

    Move();

    return 1;
}


/// method updating car physics
int CarBot::updatePhysics()
{
	// set angular velocity for rear joints
	wheel_joints[2]->setAngularVelocity(velocity);
	wheel_joints[3]->setAngularVelocity(velocity);

	// set torque for rear joints		
	wheel_joints[2]->setAngularTorque(torque);
	wheel_joints[3]->setAngularTorque(torque);

	return 1;
}