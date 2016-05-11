/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <sensor_config.h>
#include <sensor_log.h>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

using namespace config;
using std::ifstream;
using std::string;
using std::istringstream;

#define ROOT_ELEMENT	"SENSOR"
#define TEXT_ELEMENT 	"text"
#define MODEL_ID_ATTR 	"id"
#define DEFAULT_ATTR	"value"

sensor_config::sensor_config()
{
}

sensor_config& sensor_config::get_instance(void)
{
	static bool load_done = false;
	static sensor_config inst;

	if (load_done)
		return inst;

	inst.load_config();
	inst.get_device_id();
	if (!inst.m_device_id.empty())
		_I("Device ID = %s", inst.m_device_id.c_str());
	else
		_E("Failed to get Device ID");
	load_done = true;

	return inst;
}

bool sensor_config::load_config(const string& config_path)
{
	xmlDocPtr doc;
	xmlNodePtr cur;

	_D("sensor_config::load_config(\"%s\") is called!\n",config_path.c_str());

	doc = xmlParseFile(config_path.c_str());

	if (doc == NULL) {
		_E("There is no %s\n",config_path.c_str());
		return false;
	}

	cur = xmlDocGetRootElement(doc);
	if(cur == NULL) {
		_E("There is no root element in %s\n",config_path.c_str());
		xmlFreeDoc(doc);
		return false;
	}

	if(xmlStrcmp(cur->name, (const xmlChar *)ROOT_ELEMENT)) {
		_E("Wrong type document: there is no [%s] root element in %s\n",ROOT_ELEMENT,config_path.c_str());
		xmlFreeDoc(doc);
		return false;
	}

	xmlNodePtr model_list_node_ptr;
	xmlNodePtr model_node_ptr;
	xmlNodePtr element_node_ptr;
	xmlAttrPtr attr_ptr;
	char* prop = NULL;

	model_list_node_ptr = cur->xmlChildrenNode;

	while (model_list_node_ptr != NULL) {
		//skip garbage element, [text]
		if (!xmlStrcmp(model_list_node_ptr->name,(const xmlChar *)TEXT_ELEMENT)) {
			model_list_node_ptr = model_list_node_ptr->next;
			continue;
		}

		//insert Model_list to config map
		m_sensor_config[(const char*)model_list_node_ptr->name];
		_D("<%s>\n",(const char*)model_list_node_ptr->name);

		model_node_ptr = model_list_node_ptr->xmlChildrenNode;
		while (model_node_ptr != NULL){
			//skip garbage element, [text]
			if (!xmlStrcmp(model_node_ptr->name,(const xmlChar *)TEXT_ELEMENT)) {
				model_node_ptr = model_node_ptr->next;
				continue;
			}


			string model_id;
			prop = (char*)xmlGetProp(model_node_ptr,(const xmlChar*)MODEL_ID_ATTR);
			model_id = prop;
			free(prop);

			//insert Model to Model_list
			m_sensor_config[(const char*)model_list_node_ptr->name][model_id];
			_D("<%s id=\"%s\">\n",(const char*)model_list_node_ptr->name,model_id.c_str());

			element_node_ptr = model_node_ptr->xmlChildrenNode;
			while (element_node_ptr != NULL) {
				//skip garbage element, [text]
				if (!xmlStrcmp(element_node_ptr->name,(const xmlChar *)TEXT_ELEMENT)) {
					element_node_ptr = element_node_ptr->next;
					continue;
				}

				//insert Element to Model
				m_sensor_config[(const char*)model_list_node_ptr->name][model_id][(const char*)element_node_ptr->name];
				_D("<%s id=\"%s\"><%s>\n",(const char*)model_list_node_ptr->name,model_id.c_str(),(const char*)element_node_ptr->name);

				attr_ptr = element_node_ptr->properties;
				while (attr_ptr != NULL) {

					string key,value;
					key = (const char*)attr_ptr->name;
					prop = (char*)xmlGetProp(element_node_ptr,attr_ptr->name);
					value = prop;
					free(prop);

					//insert attribute to Element
					m_sensor_config[(const char*)model_list_node_ptr->name][model_id][(const char*)element_node_ptr->name][key]=value;
					_D("<%s id=\"%s\"><%s \"%s\"=\"%s\">\n",(const char*)model_list_node_ptr->name,model_id.c_str(),(const char*)element_node_ptr->name,key.c_str(),value.c_str());
					attr_ptr = attr_ptr->next;
				}


				element_node_ptr = element_node_ptr->next;
			}

			_D("\n");
			model_node_ptr = model_node_ptr->next;
		}

		_D("\n");
		model_list_node_ptr = model_list_node_ptr->next;
	}

	xmlFreeDoc(doc);
	return true;
}


bool sensor_config::get(const string& sensor_type,const string& model_id, const string& element, const string& attr, string& value)
{
	auto it_model_list = m_sensor_config.find(sensor_type);

	if (it_model_list == m_sensor_config.end())	{
		_E("There is no <%s> element\n",sensor_type.c_str());
		return false;
	}

	auto it_model = it_model_list->second.find(model_id);

	if (it_model == it_model_list->second.end()) {
		_E("There is no <%s id=\"%s\"> element\n",sensor_type.c_str(),model_id.c_str());
		return false;
	}

	auto it_element = it_model->second.find(element);

	if (it_element == it_model->second.end()) {
		_D("There is no <%s id=\"%s\"><%s> element\n",sensor_type.c_str(),model_id.c_str(),element.c_str());
		return false;
	}

	auto it_attr = it_element->second.find(attr);

	if (it_attr == it_element->second.end()) {
		_D("There is no <%s id=\"%s\"><%s \"%s\">\n",sensor_type.c_str(),model_id.c_str(),element.c_str(),attr.c_str());
		return false;
	}

	value = it_attr->second;

	return true;
}

bool sensor_config::get(const string& sensor_type, const string& model_id, const string& element, const string& attr, double& value)
{
	string str_value;

	if (get(sensor_type,model_id,element,attr,str_value) == false)
		return false;

	istringstream convert(str_value);

	if ( !(convert >> value))
		value = 0;

	return true;
}

bool sensor_config::get(const string& sensor_type, const string& model_id, const string& element, const string& attr, long& value)
{
	string str_value;

	if (get(sensor_type,model_id,element,attr,str_value) == false)
		return false;

	istringstream convert(str_value);

	if ( !(convert >> value))
		value = 0;

	return true;
}

bool sensor_config::get(const string& sensor_type, const string& model_id, const string& element, string& value)
{
	if (get(sensor_type, model_id, element, m_device_id, value))
		return true;

	if (get(sensor_type, model_id, element, DEFAULT_ATTR, value))
		return true;

	return false;
}

bool sensor_config::get(const string& sensor_type, const string& model_id, const string& element, double& value)
{
	if (get(sensor_type, model_id, element, m_device_id, value))
		return true;

	if (get(sensor_type, model_id, element, DEFAULT_ATTR, value))
		return true;

	return false;
}

bool sensor_config::get(const string& sensor_type, const string& model_id, const string& element, long& value)
{
	if (get(sensor_type, model_id, element, m_device_id, value))
		return true;

	if (get(sensor_type, model_id, element, DEFAULT_ATTR, value))
		return true;

	return false;
}

bool sensor_config::is_supported(const string& sensor_type,const string& model_id)
{
	auto it_model_list = m_sensor_config.find(sensor_type);

	if (it_model_list == m_sensor_config.end())
		return false;

	auto it_model = it_model_list->second.find(model_id);

	if (it_model == it_model_list->second.end())
		return false;

	return true;
}

bool sensor_config::get_device_id(void)
{
	const string INFO_INI_PATH = "/etc/info.ini";
	const string START_DELIMETER = "Model=";
	const string END_DELIMETER = ";";
	string line;
	ifstream in_file;
	std::size_t start_pos, end_pos;
	bool ret = false;

	in_file.open(INFO_INI_PATH);

	if (!in_file.is_open())
		return false;

	while (!in_file.eof()) {
		getline(in_file, line);
		start_pos = line.find(START_DELIMETER);

		if (start_pos == std::string::npos)
			continue;

		start_pos = start_pos + START_DELIMETER.size();
		end_pos = line.find(END_DELIMETER, start_pos);

		if (end_pos == std::string::npos)
			continue;

		m_device_id = line.substr(start_pos, end_pos - start_pos);
		ret = true;
		break;
	}

	in_file.close();

	return ret;
}
