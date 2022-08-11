#include "SettingsUi.h"

using namespace Unigine;
using namespace Unigine::Math;


SettingsUi::SettingsUi(CallbackBase* start_simulation_callback)
{
    this->start_simulation_callback = start_simulation_callback;
    GuiPtr gui = Gui::get();

    window = WidgetWindow::create(gui, "Settings", 4, 4);
    window->setFlags(Gui::ALIGN_OVERLAP | Gui::ALIGN_CENTER);
    window->setHeight(500);
    window->setWidth(400);

    for (int i = 0; i < labels_count; ++i)
    {
        label_array.push_back(WidgetLabel::create(gui, (name_array[i] + ": " + to_string(init_value[i])).c_str()));
        window->addChild(label_array[i]);
        label_array[i]->setFontSize(14);

        slider_array.push_back(WidgetSlider::create(gui, from_interval[i], to_interval[i], init_value[i]));
        window->addChild(slider_array[i], Gui::ALIGN_EXPAND);
        slider_array[i]->setButtonHeight(10);
        slider_array[i]->setButtonWidth(10);
        slider_array[i]->addCallback(Gui::CHANGED, MakeCallback(this, &SettingsUi::slider_drag, label_array[i], slider_array[i], name_array[i]));

        label_indent_array.push_back(WidgetLabel::create(gui, ""));
        window->addChild(label_indent_array[i]);
    }

    apply_button = WidgetButton::create(gui, "Apply");
    apply_button->addCallback(Gui::CLICKED, MakeCallback(this, &SettingsUi::apply_clicked));
    window->addChild(apply_button);
    apply_button->setFontSize(14);

    window->arrange();
    gui->addChild(window);
}

void SettingsUi::slider_drag(WidgetLabelPtr l, WidgetSliderPtr s, string name)
{
    l->setText((name + ": " + to_string(s->getValue())).c_str());
}

int SettingsUi::get_value(int n)
{
    if (n < this->labels_count)
    {
        return slider_array[n]->getValue();
    }
    else
    {
        Log::message("Wrong index in Ui get_value %d", n);
        return -1;
    }
}

void SettingsUi::apply_clicked()
{
    start_simulation_callback->run();
}

SettingsUi::~SettingsUi()
{
    window->deleteLater();
}
