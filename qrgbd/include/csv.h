#pragma once

/**
 * @file csv.h
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2022-01-27
 *
 * @copyright Copyright (c) 2022
 */

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace ns_csv {

  /**
   * @brief overload the operator '<<' for type 'std::stringstream' to avoid escape the space for string
   *
   * @param os the std::stringstream
   * @param str the string
   * @return std::stringstream&
   */
  static std::stringstream &operator>>(std::stringstream &os, std::string &str) {
    // don't use str << os, this will escape the space
    str = os.str();
    return os;
  }

#define THROW_EXCEPTION(where, msg) \
  throw std::runtime_error(std::string("[ error from 'libcsv'-'") + #where + "' ] " + msg)

#pragma region csv read

/**
 * @brief read all items in the ifstream
 *
 * @param ifs the input fstream
 * @param splitor the splitor
 * @param itemType the type of the item in the csv file
 * @param ... the types of the members,
 *             it's order is same as the declaration sequence of member variables.
 *
 * @return std::vector<itemType> data
 */
#define CSV_READ_IFS_ALL(ifs, splitor, itemType, ...)             \
  [](std::ifstream &ifs, char spor) -> std::vector<itemType> {    \
    std::vector<itemType> data;                                   \
    std::string strLine;                                          \
    while (std::getline(ifs, strLine)) {                          \
      auto strVec = ns_csv::ns_priv::split(strLine, spor);        \
      data.push_back(itemType{LAMBDA_PACK(strVec, __VA_ARGS__)}); \
    }                                                             \
    return data;                                                  \
  }(ifs, splitor)

/**
 * @brief read all items in the ifstream
 *
 * @param ifs the input fstream
 * @param splitor the splitor
 * @param itemType the type of the item in the csv file
 * @param ... the types of the members,
 *             it's order is same as the declaration sequence of member variables.
 *
 * @return std::pair(std::array<std::string, LabNum>, std::vector<itemType>) {header, data}
 */
#define CSV_READ_IFS_ALL_H(ifs, splitor, itemType, ...)                   \
  [](std::ifstream &ifs,                                                  \
     char spor) -> std::pair<ARRAY(__VA_ARGS__), std::vector<itemType>> { \
    std::string strLine;                                                  \
    std::getline(ifs, strLine);                                           \
    auto vec = ns_csv::ns_priv::split(strLine, spor);                     \
    ARRAY(__VA_ARGS__)                                                    \
    header;                                                               \
    for (int i = 0; i != COUNT_MACRO_VAR_ARGS(__VA_ARGS__); ++i)          \
      header.at(i) = vec.at(i);                                           \
    auto data = CSV_READ_IFS_ALL(ifs, spor, itemType, __VA_ARGS__);       \
    return {header, data};                                                \
  }(ifs, splitor)

/**
 * @brief read all items in the ifstream
 *
 * @param ifs the input fstream
 * @param splitor the splitor
 * @param itemNum the number of the items to read
 * @param itemType the type of the item in the csv file
 * @param ... the types of the members,
 *             it's order is same as the declaration sequence of member variables.
 *
 * @return std::vector<itemType> data
 */
#define CSV_READ_IFS_CER(ifs, splitor, itemNum, itemType, ...)      \
  [](std::ifstream &ifs, char spor) -> std::vector<itemType> {      \
    std::vector<itemType> data;                                     \
    std::string strLine;                                            \
    int itemCount = 0;                                              \
    while (itemCount++ < itemNum) {                                 \
      if (std::getline(ifs, strLine)) {                             \
        auto strVec = ns_csv::ns_priv::split(strLine, spor);        \
        data.push_back(itemType{LAMBDA_PACK(strVec, __VA_ARGS__)}); \
      } else                                                        \
        break;                                                      \
    }                                                               \
    return data;                                                    \
  }(ifs, splitor)

/**
 * @brief read all items in the ifstream
 *
 * @param ifs the input fstream
 * @param splitor the splitor
 * @param itemNum the number of the items to read
 * @param itemType the type of the item in the csv file
 * @param ... the types of the members,
 *             it's order is same as the declaration sequence of member variables.
 *
 * @return std::pair(std::array<std::string, LabNum>, std::vector<itemType>) {header, data}
 */
#define CSV_READ_IFS_CER_H(ifs, splitor, itemNum, itemType, ...)             \
  [](std::ifstream &ifs,                                                     \
     char spor) -> std::pair<ARRAY(__VA_ARGS__), std::vector<itemType>> {    \
    std::string strLine;                                                     \
    std::getline(ifs, strLine);                                              \
    auto vec = ns_csv::ns_priv::split(strLine, spor);                        \
    ARRAY(__VA_ARGS__)                                                       \
    header;                                                                  \
    for (int i = 0; i != COUNT_MACRO_VAR_ARGS(__VA_ARGS__); ++i)             \
      header.at(i) = vec.at(i);                                              \
    auto data = CSV_READ_IFS_CER(ifs, spor, itemNum, itemType, __VA_ARGS__); \
    return {header, data};                                                   \
  }(ifs, splitor)

/**
 * @brief read all items in the ifstream
 *
 * @param fileName the file name
 * @param splitor the splitor
 * @param itemType the type of the item in the csv file
 * @param ... the types of the members,
 *             it's order is same as the declaration sequence of member variables.
 *
 * @return std::vector<itemType> data
 */
#define CSV_READ_FILE(fileName, splitor, itemType, ...)             \
  [](const std::string &name, char spor) -> std::vector<itemType> { \
    std::ifstream ifs(name);                                        \
    return CSV_READ_IFS_ALL(ifs, spor, itemType, __VA_ARGS__);      \
  }(fileName, splitor)

/**
 * @brief read all items in the ifstream
 *
 * @param fileName the file name
 * @param splitor the splitor
 * @param itemType the type of the item in the csv file
 * @param ... the types of the members,
 *             it's order is same as the declaration sequence of member variables.
 *
 * @return std::pair(std::array<std::string, LabNum>, std::vector<itemType>) {header, data}
 */
#define CSV_READ_FILE_H(fileName, splitor, itemType, ...)                 \
  [](const std::string &name,                                             \
     char spor) -> std::pair<ARRAY(__VA_ARGS__), std::vector<itemType>> { \
    std::ifstream ifs(name);                                              \
    std::string strLine;                                                  \
    std::getline(ifs, strLine);                                           \
    auto vec = ns_csv::ns_priv::split(strLine, spor);                     \
    ARRAY(__VA_ARGS__)                                                    \
    header;                                                               \
    for (int i = 0; i != COUNT_MACRO_VAR_ARGS(__VA_ARGS__); ++i)          \
      header.at(i) = vec.at(i);                                           \
    auto data = CSV_READ_IFS_ALL(ifs, spor, itemType, __VA_ARGS__);       \
    return {header, data};                                                \
  }(fileName, splitor)

#pragma endregion

#pragma region csv write

/**
 * @brief write data to a csv file
 *
 * @param ofs the out fstream
 * @param data the data array
 * @param splitor the splitor
 * @param ... the [methods | member name] to get members from a item
 *
 * @return void
 */
#define CSV_WRITE_OFS(ofs, data, splitor, ...)                         \
  [](std::ofstream &ofs, const decltype(data) &d, char spor) -> void { \
    for (const auto &elem : d)                                         \
      ns_csv::ns_priv::__print__(ofs, spor, __VA_ARGS__);              \
    return;                                                            \
  }(ofs, data, splitor)

/**
 * @brief write data to a csv file
 *
 * @param osftream the out fstream
 * @param header the header labels
 * @param data the data array
 * @param splitor the splitor
 * @param ... the [methods | member name] to get members from a item
 *
 * @return void
 */
#define CSV_WRITE_OFS_H(ofs, header, data, splitor, ...)  \
  [](std::ofstream &ofs, const ARRAY(__VA_ARGS__) & h,    \
     const decltype(data) &d, char spor) -> void {        \
    ns_csv::ns_priv::__print__(ofs, spor, h);             \
    for (const auto &elem : d)                            \
      ns_csv::ns_priv::__print__(ofs, spor, __VA_ARGS__); \
    return;                                               \
  }(ofs, header, data, splitor)

/**
 * @brief write data to a csv file
 *
 * @param fileName the file name
 * @param data the data array
 * @param splitor the splitor
 * @param ... the [methods | member name] to get members from a item
 *
 * @return void
 */
#define CSV_WRITE_FILE(fileName, data, splitor, ...)                        \
  [](const std::string &name, const decltype(data) &d, char spor) -> void { \
    std::ofstream ofs(name);                                                \
    for (const auto &elem : d)                                              \
      ns_csv::ns_priv::__print__(ofs, spor, __VA_ARGS__);                   \
    ofs.close();                                                            \
    return;                                                                 \
  }(fileName, data, splitor)

/**
 * @brief write data to a csv file
 *
 * @param fileName the file name
 * @param header the header labels
 * @param data the data array
 * @param splitor the splitor
 * @param ... the [methods | member name] to get members from a item
 *
 * @return void
 */
#define CSV_WRITE_FILE_H(fileName, header, data, splitor, ...) \
  [](const std::string &name, const ARRAY(__VA_ARGS__) & h,    \
     const decltype(data) &d, char spor) -> void {             \
    std::ofstream ofs(name);                                   \
    ns_csv::ns_priv::__print__(ofs, spor, h);                  \
    for (const auto &elem : d)                                 \
      ns_csv::ns_priv::__print__(ofs, spor, __VA_ARGS__);      \
    ofs.close();                                               \
    return;                                                    \
  }(fileName, header, data, splitor)

#pragma endregion

#pragma region help functions
  namespace ns_priv {

    /**
     * \brief a function to split a string to some string elements according the splitor
     *
     * \param str the string to be splited \param splitor the splitor char
     * \param ignoreEmpty whether ignoring the empty string element or not
     *
     * \return the splited string vector
     */
    static std::vector<std::string> split(const std::string &str, char splitor,
                                          bool ignoreEmpty = true) {
      std::vector<std::string> vec;
      auto iter = str.cbegin();
      while (true) {
        auto pos = std::find(iter, str.cend(), splitor);
        auto elem = std::string(iter, pos);
        if ((!ignoreEmpty) || (ignoreEmpty && !elem.empty()))
          vec.push_back(elem);
        if (pos == str.cend())
          break;
        iter = ++pos;
      }
      return vec;
    }

    /**
     * @brief used to cast types
     */
    static std::stringstream strStream;

    template <std::size_t Size>
    void __print__(std::ofstream &ofs, char splitor,
                   const std::array<std::string, Size> &header) {
      for (int i = 0; i != Size - 1; ++i)
        ofs << header.at(i) << splitor;
      ofs << header.at(Size - 1) << '\n';
    }

    template <typename ArgvType>
    void __print__(std::ofstream &ofs, char splitor, const ArgvType &argv) {
      ofs << argv << '\n';
      return;
    }

    /**
     * @brief print the argvs with template param list
     *
     * @tparam ArgvType
     * @tparam ArgvsType
     * @param ofs the output file stream
     * @param splitor the splitor
     * @param argv one of the argvs
     * @param argvs the else argvs
     */
    template <typename ArgvType, typename... ArgvsType>
    void __print__(std::ofstream &ofs, char splitor, const ArgvType &argv,
                   const ArgvsType &...argvs) {
      ofs << argv << splitor;
      return __print__(ofs, splitor, argvs...);
    }

  } // namespace ns_priv
#pragma endregion

#pragma region csv reader and write

  namespace ns_priv {
    class Reader {
    public:
      Reader(std::ifstream *ifs) : _ifs(ifs) {}

      virtual ~Reader() {}

      /**
       * @brief get next std::string vector and assign to the elems
       */
      template <typename... ElemTypes>
      bool readLine(char splitor = ',', ElemTypes &...elems) {
        std::string str;
        if (std::getline(*(this->_ifs), str)) {
          auto strVec = ns_priv::split(str, splitor);
          this->parse(strVec, 0, elems...);
          return true;
        }
        return false;
      }

    protected:
      template <typename ElemType, typename... ElemTypes>
      void parse(const std::vector<std::string> &strVec, std::size_t index,
                 ElemType &elem, ElemTypes &...elems) {
        std::stringstream stream;
        stream << strVec.at(index);
        stream >> elem;
        return this->parse(strVec, index + 1, elems...);
      }

      void parse(const std::vector<std::string> &strVec, std::size_t index) {
        return;
      }

    protected:
      /**
       * @brief the input file stream
       */
      std::ifstream *_ifs;

    private:
      Reader() = delete;
      Reader(const Reader &) = delete;
      Reader(Reader &&) = delete;
      Reader &operator=(const Reader &) = delete;
      Reader &operator=(Reader &&) = delete;
    };

    class StreamReader : public Reader {

    public:
      StreamReader(std::ifstream &ifs) : Reader(&ifs) {
        if (!this->_ifs->is_open()) {
          THROW_EXCEPTION(CSVReader, "the file stream may be invalid");
        }
      }

      virtual ~StreamReader() override {}
    };

    class FileReader : public Reader {

    public:
      FileReader(const std::string &filename) : Reader(new std::ifstream(filename)) {
        if (!this->_ifs->is_open()) {
          THROW_EXCEPTION(CSVReader, "the file name may be invalid");
        }
      }

      virtual ~FileReader() override {
        delete this->_ifs;
      }
    };

  } // namespace ns_priv

  class CSVReader {
  public:
    using Ptr = std::shared_ptr<ns_priv::Reader>;

  public:
    /**
     * @brief create a file reader pointer
     *
     * @param filename the name of the csv file
     * @return Ptr
     */
    static Ptr create(const std::string &filename) {
      return std::make_shared<ns_priv::FileReader>(filename);
    }

    /**
     * @brief create a input file stream reader pointer
     *
     * @param ifs input file stream
     * @return Ptr
     */
    static Ptr create(std::ifstream &ifs) {
      return std::make_shared<ns_priv::StreamReader>(ifs);
    }
  };

  namespace ns_priv {
    class Writer {
    public:
      Writer(std::ofstream *ofs) : _ofs(ofs) {}

      virtual ~Writer() {}

      /**
       * @brief use variable template parameters to write any num arguements
       */
      template <typename... Types>
      void writeLine(char splitor, const Types &...argvs) {
        ns_priv::__print__(*(this->_ofs), splitor, argvs...);
        return;
      }

      void setPrecision(std::size_t prec) {
        *(this->_ofs) << std::fixed << std::setprecision(prec);
        return;
      }

    protected:
      /**
       * @brief the output file stream
       */
      std::ofstream *_ofs;

    private:
      Writer() = delete;
      Writer(const Writer &) = delete;
      Writer(Writer &&) = delete;
      Writer &operator=(const Writer &) = delete;
      Writer &operator=(Writer &&) = delete;
    };

    class StreamWriter : public Writer {
    public:
      StreamWriter(std::ofstream &ofs) : Writer(&ofs) {
        if (!this->_ofs->is_open()) {
          THROW_EXCEPTION(CSVWriter, "the file stream may be invalid");
        }
      }

      virtual ~StreamWriter() override {}
    };

    class FileWriter : public Writer {
    public:
      FileWriter(const std::string &filename) : Writer(new std::ofstream(filename)) {
        if (!this->_ofs->is_open()) {
          THROW_EXCEPTION(CSVWriter, "the file name may be invalid");
        }
      }

      virtual ~FileWriter() override {
        delete this->_ofs;
      }
    };

  } // namespace ns_priv

  class CSVWriter {
  public:
    using Ptr = std::shared_ptr<ns_priv::Writer>;

  public:
    /**
     * @brief create a file writer pointer
     *
     * @param filename the name of the csv file
     * @return Ptr
     */
    static Ptr create(const std::string &filename) {
      return std::make_shared<ns_priv::FileWriter>(filename);
    }

    /**
     * @brief create a output file stream writer pointer
     *
     * @param ifs output file stream
     * @return Ptr
     */
    static Ptr create(std::ofstream &ofs) {
      return std::make_shared<ns_priv::StreamWriter>(ofs);
    }
  };

#pragma endregion

#pragma region help macroes

#define MACRO_VAR_ARGS_IMPL_COUNT(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, N, \
                                  ...) N

#define COUNT_MACRO_VAR_ARGS(...) \
  MACRO_VAR_ARGS_IMPL_COUNT(__VA_ARGS__, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)

#define MACRO_COMBINE_2(MACRO, ARGS_COUNT) MACRO##ARGS_COUNT
#define MACRO_COMBINE_1(MACRO, ARGS_COUNT) MACRO_COMBINE_2(MACRO, ARGS_COUNT)
#define MACRO_COMBINE(MACRO, ARGS_COUNT) MACRO_COMBINE_1(MACRO, ARGS_COUNT)

#define ARRAY(...) std::array<std::string, COUNT_MACRO_VAR_ARGS(__VA_ARGS__)>

/**
 * @brief the method to get element's data when writing data to csv file
 */
#define CSV_ELEM(method) elem.method

/**
 * @brief organize the cvs file headers
 */
#define CSV_HEADER(...) \
  ARRAY(__VA_ARGS__) { __VA_ARGS__ }

#pragma endregion

#pragma region read macroes

/**
 * @brief use to cast string type to others
 */
#define STR_TRANS(strStream, str, val) \
  strStream << str;                    \
  strStream >> val;                    \
  strStream.clear();                   \
  strStream.str("")

/**
 * @brief generate the lambda function to trans the string to the type 'dtsType'
 */
#define LAMBDA_TRANS(srcStr, dstType)                                   \
  [](std::stringstream &strStream, const std::string &str) -> dstType { \
    dstType val;                                                        \
    STR_TRANS(strStream, str, val);                                     \
    return val;                                                         \
  }(ns_csv::ns_priv::strStream, srcStr)

/**
 * @brief generate the param list with lambda
 */
#define LAMBDA_PACK(strVec, ...)                                 \
  MACRO_COMBINE(LAMBDA_PACK_, COUNT_MACRO_VAR_ARGS(__VA_ARGS__)) \
  (strVec, __VA_ARGS__)

#define LAMBDA_PACK_10(strVec, dstType, ...)            \
  LAMBDA_TRANS(strVec.at(strVec.size() - 10), dstType), \
      LAMBDA_PACK_9(strVec, __VA_ARGS__)
#define LAMBDA_PACK_9(strVec, dstType, ...)            \
  LAMBDA_TRANS(strVec.at(strVec.size() - 9), dstType), \
      LAMBDA_PACK_8(strVec, __VA_ARGS__)
#define LAMBDA_PACK_8(strVec, dstType, ...)            \
  LAMBDA_TRANS(strVec.at(strVec.size() - 8), dstType), \
      LAMBDA_PACK_7(strVec, __VA_ARGS__)
#define LAMBDA_PACK_7(strVec, dstType, ...)            \
  LAMBDA_TRANS(strVec.at(strVec.size() - 7), dstType), \
      LAMBDA_PACK_6(strVec, __VA_ARGS__)
#define LAMBDA_PACK_6(strVec, dstType, ...)            \
  LAMBDA_TRANS(strVec.at(strVec.size() - 6), dstType), \
      LAMBDA_PACK_5(strVec, __VA_ARGS__)
#define LAMBDA_PACK_5(strVec, dstType, ...)            \
  LAMBDA_TRANS(strVec.at(strVec.size() - 5), dstType), \
      LAMBDA_PACK_4(strVec, __VA_ARGS__)
#define LAMBDA_PACK_4(strVec, dstType, ...)            \
  LAMBDA_TRANS(strVec.at(strVec.size() - 4), dstType), \
      LAMBDA_PACK_3(strVec, __VA_ARGS__)
#define LAMBDA_PACK_3(strVec, dstType, ...)            \
  LAMBDA_TRANS(strVec.at(strVec.size() - 3), dstType), \
      LAMBDA_PACK_2(strVec, __VA_ARGS__)
#define LAMBDA_PACK_2(strVec, dstType, ...)            \
  LAMBDA_TRANS(strVec.at(strVec.size() - 2), dstType), \
      LAMBDA_PACK_1(strVec, __VA_ARGS__)
#define LAMBDA_PACK_1(strVec, dstType) \
  LAMBDA_TRANS(strVec.at(strVec.size() - 1), dstType)

#pragma endregion

} // namespace ns_csv
