#pragma once
#include <UnigineComponentSystem.h>
#include "UniginePrimitives.h"
#include <UnigineObjects.h>
#include "UnigineGame.h"
#include <UnigineRender.h>
#include <UnigineViewport.h>
#include <vector>
#include <string>

using namespace Unigine;
using namespace std;

class Eyes
{
public:
    Eyes();
    ~Eyes();
    vector<int> get_image();
    NodeDummyPtr get_eyes_node();
    Unigine::PlayerSpectatorPtr get_camera();
    void create_camera();
    void create_node();
private:
    void render();

    Unigine::PlayerSpectatorPtr player;
    Unigine::NodeDummyPtr eyes_node;
    Unigine::TexturePtr screenshot;
    Unigine::ViewportPtr viewport;
    Unigine::RenderTargetPtr rendertarget;

    int red_mask[30][30], green_mask[30][30], blue_mask[30][30];
    int result_image_size = 2700;
};

