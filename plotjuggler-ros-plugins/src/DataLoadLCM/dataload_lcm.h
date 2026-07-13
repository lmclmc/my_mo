#ifndef DATALOAD_LCM_H
#define DATALOAD_LCM_H

#include <QObject>
#include <QtPlugin>
#include <QSettings>

#include <PlotJuggler/dataloader_base.h>
#include "parser_configuration.h"
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm_coretypes.h>
#include "lcmgen.h"
#include "tokenize.h"

class DataLoadLCM : public PJ::DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.ROS2DataLoader")
    Q_INTERFACES(PJ::DataLoader)

public:
    DataLoadLCM();

    virtual const std::vector<const char*>& compatibleFileExtensions() const override;

    virtual bool readDataFromFile(PJ::FileLoadInfo* fileload_info, PJ::PlotDataMapRef& destination) override;

    virtual const char* name() const override
    {
      return "DataLoad LCM";
    }

    virtual ~DataLoadLCM() override = default;

    virtual bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

    virtual bool xmlLoadState(const QDomElement& parent_element) override;

private:
    std::map<std::string, lcm_struct_t*> lcm_type_table;
    PJ::PlotDataMapRef* plot_data = nullptr;

    std::vector<const char*> _extensions;

    PJ::RosParserConfig _config;

    std::vector<std::pair<QString, QString>> getAndRegisterAllTopics();

    void saveDefaultSettings();

    void loadDefaultSettings();

    int decode_type(const lcm::LogEvent* event, int, const std::string&, const std::string&, int deep= 0);
};

#endif  // DATALOAD_ROS2_H
