//
// Created by clemens on 19.11.15.
//

#ifndef TUG_TESTING_PARAMTERHELPER_H
#define TUG_TESTING_PARAMTERHELPER_H

#include <set>
#include <string>
#include <XmlRpcValue.h>
#include <vector>
#include <map>

class ParameterHelper
{
    std::set<std::string> used_parameters_;
    std::string tmp_param_prefix_;

    std::string createTmpParamName(const std::string& key);

public:
    ParameterHelper();
    explicit ParameterHelper(std::string tmp_param_prefix);
    virtual ~ParameterHelper();

    void setTmpParameter(const std::string& key, const XmlRpc::XmlRpcValue& v);
    void setTmpParameter(const std::string& key, const std::string& s);
    void setTmpParameter(const std::string& key, const char* s);
    void setTmpParameter(const std::string& key, double d);
    void setTmpParameter(const std::string& key, int i);
    void setTmpParameter(const std::string& key, bool b);
    void setTmpParameter(const std::string& key, const std::vector<std::string>& vec);
    void setTmpParameter(const std::string& key, const std::vector<double>& vec);
    void setTmpParameter(const std::string& key, const std::vector<float>& vec);
    void setTmpParameter(const std::string& key, const std::vector<int>& vec);
    void setTmpParameter(const std::string& key, const std::vector<bool>& vec);
    void setTmpParameter(const std::string& key, const std::map<std::string, std::string>& map);
    void setTmpParameter(const std::string& key, const std::map<std::string, double>& map);
    void setTmpParameter(const std::string& key, const std::map<std::string, float>& map);
    void setTmpParameter(const std::string& key, const std::map<std::string, int>& map);
    void setTmpParameter(const std::string& key, const std::map<std::string, bool>& map);

    XmlRpc::XmlRpcValue getTmpParams();

    void setParameter(const std::string& key, const XmlRpc::XmlRpcValue& v);
    void setParameter(const std::string& key, const std::string& s);
    void setParameter(const std::string& key, const char* s);
    void setParameter(const std::string& key, double d);
    void setParameter(const std::string& key, int i);
    void setParameter(const std::string& key, bool b);
    void setParameter(const std::string& key, const std::vector<std::string>& vec);
    void setParameter(const std::string& key, const std::vector<double>& vec);
    void setParameter(const std::string& key, const std::vector<float>& vec);
    void setParameter(const std::string& key, const std::vector<int>& vec);
    void setParameter(const std::string& key, const std::vector<bool>& vec);
    void setParameter(const std::string& key, const std::map<std::string, std::string>& map);
    void setParameter(const std::string& key, const std::map<std::string, double>& map);
    void setParameter(const std::string& key, const std::map<std::string, float>& map);
    void setParameter(const std::string& key, const std::map<std::string, int>& map);
    void setParameter(const std::string& key, const std::map<std::string, bool>& map);
};


#endif //TUG_TESTING_PARAMTERHELPER_H
