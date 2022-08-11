#include "Eyes.h"


Eyes::Eyes()
{
    viewport = Viewport::create();
}

void Eyes::create_node()
{
    eyes_node = NodeDummy::create();
}

void Eyes::create_camera()
{
    player = PlayerSpectator::create();

    player->setCollisionMask(0);
    
    player->setViewDirection(Unigine::Math::vec3(1,0,-0.4));
    
    player->setPosition(Unigine::Math::vec3(0.0f, 0.f, 4.0f));
    player->setControlled(0);
    Game::setEnabled(1);
}

NodeDummyPtr Eyes::get_eyes_node()
{
    return this->eyes_node;
}

Unigine::PlayerSpectatorPtr Eyes::get_camera()
{
    return this->player;
}

void Eyes::render()
{
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

    // screenshot_image->save(("screenshots/screenshot" + to_string(Game::getRandomInt(0, 1000)) + ".png").c_str());

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

vector<int> Eyes::get_image()
{
    vector<int> res_img;
    this->render();
    for (int i = 0; i < 30; ++i)
        for (int j = 0; j < 30; ++j) res_img.push_back(red_mask[i][j]);
    for (int i = 0; i < 30; ++i)
        for (int j = 0; j < 30; ++j) res_img.push_back(green_mask[i][j]);
    for (int i = 0; i < 30; ++i)
        for (int j = 0; j < 30; ++j) res_img.push_back(blue_mask[i][j]);
    return res_img;

}
Eyes::~Eyes()
{
    this->eyes_node.deleteLater();
    player->deleteLater();
}
