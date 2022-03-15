#ifndef CONFIG_H
#define	CONFIG_H
#include <string>
#include "tinyxml2.h"

class Config
{
    public:
        Config();
        //Config(const Config& orig);
        ~Config();
        bool getConfig(const char *FileName);

        double*         SearchParams;
        std::string*    LogParams;
        unsigned int    N;
        int             searchType;
        int             maxTime = 1;
        double          maxEdgeLength = 1.0;
        int             seed = 0;


    private:
        bool getValueFromText(tinyxml2::XMLElement *elem, const char *name, const char *typeName, void *field);
        bool getValueFromAttribute(tinyxml2::XMLElement *elem, const char *elemName,
                                          const char *attrName, const char *typeName, void *field);
        bool getText(tinyxml2::XMLElement *elem, const char *name, std::string &field);
        tinyxml2::XMLElement* getChild(tinyxml2::XMLElement *elem, const char *name, bool printError = true);
};


#endif

