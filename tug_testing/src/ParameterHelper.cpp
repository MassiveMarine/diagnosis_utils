//
// Created by clemens on 19.11.15.
//

#include <tug_testing/ParameterHelper.h>
#include <ros/ros.h>

ParameterHelper::ParameterHelper() : tmp_param_prefix_("/test")
{ }

ParameterHelper::ParameterHelper(std::string tmp_param_prefix) : tmp_param_prefix_(tmp_param_prefix)
{ }

ParameterHelper::~ParameterHelper()
{
  for(std::set<std::string>::iterator it = used_parameters_.begin(); it != used_parameters_.end(); ++it)
    ros::param::del(*it);
}

std::string ParameterHelper::createTmpParamName(const std::string& key)
{
  if(key.find_first_of("/") == 0)
    return tmp_param_prefix_ + key;

  return tmp_param_prefix_ + "/" + key;
}

void ParameterHelper::setTmpParameter(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, v);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::string& s)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, s);
}

void ParameterHelper::setTmpParameter(const std::string& key, const char* s)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, s);
}

void ParameterHelper::setTmpParameter(const std::string& key, double d)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, d);
}

void ParameterHelper::setTmpParameter(const std::string& key, int i)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, i);
}

void ParameterHelper::setTmpParameter(const std::string& key, bool b)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, b);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::vector<std::string>& vec)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, vec);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::vector<double>& vec)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, vec);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::vector<float>& vec)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, vec);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::vector<int>& vec)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, vec);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::vector<bool>& vec)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, vec);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::map<std::string, std::string>& map)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, map);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::map<std::string, double>& map)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, map);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::map<std::string, float>& map)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, map);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::map<std::string, int>& map)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, map);
}

void ParameterHelper::setTmpParameter(const std::string& key, const std::map<std::string, bool>& map)
{
  std::string param_name = createTmpParamName(key);
  used_parameters_.insert(param_name);
  ros::param::set(param_name, map);
}

XmlRpc::XmlRpcValue ParameterHelper::getTmpParams()
{
  XmlRpc::XmlRpcValue params;
  ros::param::get(tmp_param_prefix_, params);

  return params;
}

void ParameterHelper::setParameter(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, v);
}

void ParameterHelper::setParameter(const std::string& key, const std::string& s)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, s);
}

void ParameterHelper::setParameter(const std::string& key, const char* s)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, s);
}

void ParameterHelper::setParameter(const std::string& key, double d)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, d);
}

void ParameterHelper::setParameter(const std::string& key, int i)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, i);
}

void ParameterHelper::setParameter(const std::string& key, bool b)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, b);
}

void ParameterHelper::setParameter(const std::string& key, const std::vector<std::string>& vec)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, vec);
}

void ParameterHelper::setParameter(const std::string& key, const std::vector<double>& vec)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, vec);
}

void ParameterHelper::setParameter(const std::string& key, const std::vector<float>& vec)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, vec);
}

void ParameterHelper::setParameter(const std::string& key, const std::vector<int>& vec)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, vec);
}

void ParameterHelper::setParameter(const std::string& key, const std::vector<bool>& vec)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, vec);
}

void ParameterHelper::setParameter(const std::string& key, const std::map<std::string, std::string>& map)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, map);
}

void ParameterHelper::setParameter(const std::string& key, const std::map<std::string, double>& map)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, map);
}

void ParameterHelper::setParameter(const std::string& key, const std::map<std::string, float>& map)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, map);
}

void ParameterHelper::setParameter(const std::string& key, const std::map<std::string, int>& map)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, map);
}

void ParameterHelper::setParameter(const std::string& key, const std::map<std::string, bool>& map)
{
  std::string param_name = key;
  used_parameters_.insert(param_name);
  ros::param::set(param_name, map);
}
