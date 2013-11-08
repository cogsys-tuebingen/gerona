#ifndef CONFIG_FILE_READER
#define CONFIG_FILE_READER

#include <map>
#include <string>
#include "Global.h"
/**
* @class ConfigFileReader
*
* @brief The ConfigFileReader class provides an interface for reading (and writing) configuration files.
*
* Example of a configuration file:
*
* # Comment
* key1 = value1
* key2 = value2
* key3 = "testValue"
* #If quotas exist, whitespaces are not ignored - only the value may be surrounded with quotas!
* changed whitespace around keys is trimmed
* @author Stefan Laible
*/
class ConfigFileReader
{
public:
    ConfigFileReader();
    ConfigFileReader(std::string filename);

    void Load(std::string filename);
    void Save(std::string filename);
    void SaveMatlab(const std::string& filename, const std::string& base_key_spec);
    void Clear();

    /**
    *  @brief  Get the value associated to a given key
    *  @param  key  The given key
    *  @param  def  Default value, if key doesn't exist
    *  @return The value
    */
    bool GetBool(std::string key, bool def = false) const;
    int GetInt(std::string key, int def = 0) const;
    void GetIntList(std::string key, IList& values, IList& def) const;
    double GetDouble(std::string key, double def = 0.0) const;
    std::string GetString(std::string key, std::string def = "") const;
    std::map<std::string, std::string> GetMap() const;

    /**
    * @brief Set the value associated to a given key
    * @param key   The given key
    * @param value The value
    */
    void SetBool(std::string key, bool value);
    void SetInt(std::string key, int value);
    void SetIntList(std::string key, const IList& values);
    void SetDouble(std::string key, double value);
    void SetString(std::string key, std::string value);

    /**
      checks if an entry with given key exists.
      if fullMatch==false
      checks if a key starting with parameter exists
      */
    bool KeyExists (const std::string& key, bool fullMatch = true);


private:
    std::map<std::string, std::string> mMap;
};

#endif
