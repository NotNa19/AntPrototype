#include "PeriodSimulation.h"

void create_food(int num)
{
    for (int i = 0; i < num; ++i)
    {
        auto temp_food = NodeDummy::create();
        temp_food->setWorldPosition(vec3(0, 0, 30));
        temp_food->setProperty("Food");
    }
}

PeriodSimulation::PeriodSimulation(AppWorldLogic* AWL, vector<BrainLifetime> brains, vector<int> values_from_settings)
{
    Log::message("Start new period \n");

    this->spawn_limit = values_from_settings[0];
    this->start_num_of_ants = values_from_settings[1];
    this->next_num_of_ants = values_from_settings[2];
    this->ants_in_period = values_from_settings[3];
    this->initial_food_count = values_from_settings[4];
    this->food_per_minute = values_from_settings[5];

    counter = new Counter(this, ants_in_period, next_num_of_ants/2);
    spawner = new AnimalSpawner(counter, spawn_limit, &cameras);
    label = WidgetLabel::create(Gui::get());
    Gui::get()->addChild(label, Gui::ALIGN_OVERLAP);

    end_period_callback = MakeCallback(AWL, &AppWorldLogic::end_period_callback);

    this->set_best_period_brains(brains);
    this->init();
}

void PeriodSimulation::destroy_food_callback(NodePtr ent_node)
{
    if (string(ent_node->getName()) == string("f"))
    {
        ent_node->deleteLater();
    }
}

void PeriodSimulation::set_best_period_brains(vector<BrainLifetime> best_brains)
{
    Log::message("Set in period\n");
    for (int i = 0; i < this->next_num_of_ants/2; i++)
    {
        counter->best_period_brains.push_back(best_brains[i]);
    }
}

void PeriodSimulation::end_period()
{
    food_destroyer = WorldTrigger::create(vec3(500, 500, 10));
    food_destroyer->addEnterCallback(MakeCallback(this, &PeriodSimulation::destroy_food_callback));
    food_destroyer->setWorldPosition(vec3(0,0,0));

    period_end_flag = 1;
    spawner->off();
}

PeriodSimulation::~PeriodSimulation()
{
    food_destroyer->deleteLater();
    end_period_callback->run(counter->best_period_brains);
    delete counter;
    delete spawner;   
    end_period_callback->~CallbackBase(); 
    label->deleteLater();
    this->shutdown();
    Log::message("End period \n");
}

int PeriodSimulation::init()
{
    Game::setSeed(std::time(nullptr));
    label->setPosition(10, 10);
    label->setFontSize(24);
    label->setFontOutline(1);

    start_time = t.getTime();
    time_food = t.getTime();
    time_spawn = t.getTime();

    create_food(initial_food_count);

    if (playerable_character)
        spawner->create_player_ant(vec3(Game::getRandomFloat(-170, 170), Game::getRandomFloat(-170, 170), 2.4));

    for (int i = 0; i < start_num_of_ants / 2; ++i)
    {
        spawner->create_ant(vec3(Game::getRandomFloat(-170, 170), Game::getRandomFloat(-170, 170), 2.4), NULL, 0);
    }
    for (int i = 0; i < counter->best_period_brains.size(); ++i)
    {
        spawner->create_ant(vec3(Game::getRandomFloat(-170, 170), Game::getRandomFloat(-170, 170), 2.4), counter->best_period_brains[i].brain, 0);
    }


    return 1;
}

int PeriodSimulation::update()
{
    label->setText(String::format(
        "Current Time: %d\n"
        "Last spawn: %d \n"
        "Spawn reload: %d \n"
        "Ants for all time:: %d \n"
        "Now %d ants \n",
        (t.getTime() - start_time) / 1000000,
        (time_spawn - start_time) / 1000000,
        time_spawn_change / 1000000,
        counter->total_ants,
        counter->current_ants));

    if ((t.getTime() - time_food > time_food_spawn))
    {
        time_food = t.getTime();
        create_food(food_per_minute/2);

    }

    if ((t.getTime() - time_spawn > time_spawn_change))
    {
        Log::message("comp: %d \n", ComponentSystem::get()->getNumComponents());
        time_spawn = t.getTime();
        for (int i = 0; i < next_num_of_ants / 2; ++i)
        {
            spawner->create_ant(vec3(Game::getRandomFloat(-170, 170), Game::getRandomFloat(-170, 170), 2.4), NULL, 0);
        }
        for (int i = 0; i < counter->best_period_brains.size(); ++i)
        {
            spawner->create_ant(vec3(Game::getRandomFloat(-170, 170), Game::getRandomFloat(-170, 170), 2.4), counter->best_period_brains[i].brain, 0);
        }
    }

    if (period_end_flag && counter->current_ants == 0)
    {
        this->~PeriodSimulation();
    }

    if (first_view && !current_camera)
    {
        while (cameras.size() != 0)
        {
            if (cameras[cameras.size() - 1])
            {
                current_camera = cameras[cameras.size() - 1];
                Game::setPlayer(current_camera);
                break;
            }
            else
            {
                cameras.pop_back();
            }
        }
    }
    

    return 1;
}
