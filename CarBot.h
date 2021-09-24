// car.h

#include "UnigineGame.h"
#include "UnigineEngine.h"
#include "UnigineObjects.h"
#include "UniginePrimitives.h"
#include <UnigineCamera.h>

#include <UnigineApp.h>
#include <UnigineLogic.h>
#include <UnigineConsole.h>
#include <UnigineViewport.h>
#include <UnigineCallback.h>
#include <UnigineRender.h>
#include <UnigineWidgets.h>
#include <UnigineImage.h>
#include <UnigineWorld.h>
#include <string>
/// Car class definition
class CarBot
{
public:
	// car parameters
	float frame_width;
	float frame_height;
	float frame_length;
	float wheel_radius;
	float wheel_width;

	// initialization of movement parameters
	float angle = 0.0f;
	float velocity = 0.0f;
	float torque = 0.0f;

	CarBot() {	}
	~CarBot() {	}

	int update();
    int init(int num,Unigine::Math::vec3 position, float blength, float bwidth, float bheight, float wradius, float wwidth, float wmass, const Unigine::Math::Mat4& transform);
	int updatePhysics();
    void Move();


private:

	// car elements
	Unigine::ObjectMeshDynamicPtr car_frame;
	Unigine::ObjectMeshDynamicPtr wheels[4];
	Unigine::JointWheelPtr wheel_joints[4];
	Unigine::ControlsPtr controls;
    int car_num;
    //Unigine::PlayerDummyPtr player;
    Unigine::PlayerPersecutorPtr player;
    Unigine::TexturePtr screenshot;
    Unigine::ViewportPtr viewport;
    Unigine::RenderTargetPtr rendertarget;
    long long time_change = 500000, time;
    Unigine::Timer t;
    int im_count;
    int red_mask[30][30], green_mask[30][30], blue_mask[30][30];
    bool direction[4]; //0-forward, 1-back, 2-left, 3-right
};