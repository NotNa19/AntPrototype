#pragma once
#include <UnigineLogic.h>
#include <UnigineComponentSystem.h>
#include "UnigineApp.h"
#include <UnigineEngine.h>
#include <UnigineGui.h>
#include <UnigineLogic.h>
#include <UnigineWidgets.h>
#include <string>
#include <vector>
using namespace std;

class SettingsUi
{
public:
    SettingsUi(Unigine::CallbackBase* start_simulation_callback);
    ~SettingsUi();
    int get_value(int n);
    int labels_count = 6;

private:

    void slider_drag(Unigine::WidgetLabelPtr l, Unigine::WidgetSliderPtr s, std::string name);
    void apply_clicked();

    vector<Unigine::WidgetLabelPtr> label_array;
    vector<Unigine::WidgetLabelPtr> label_indent_array;
    vector<Unigine::WidgetSliderPtr> slider_array;
    

    vector<string> name_array = { "Spawn limit","Start ants count", "Respawn ants count", "Ants for period",
        "Initial food count", "Food per minute" };
    vector<int> from_interval = { 0, 0, 0, 50,
    0, 0};
    vector<int> to_interval = { 100, 50, 50, 1000,
    10000, 1000};
    vector<int> init_value = { 40, 20, 20, 500,
    2000, 200};


    Unigine::WidgetButtonPtr apply_button;
    Unigine::CallbackBase* start_simulation_callback;
    Unigine::WidgetWindowPtr window;
};

