/**
 *
 *  Straightlink.h
 *  DO NOT EDIT. This file is generated by drogon_ctl
 *
 */

#pragma once
#include <drogon/orm/Result.h>
#include <drogon/orm/Row.h>
#include <drogon/orm/Field.h>
#include <drogon/orm/SqlBinder.h>
#include <drogon/orm/Mapper.h>
#include <drogon/orm/BaseBuilder.h>
#ifdef __cpp_impl_coroutine
#include <drogon/orm/CoroMapper.h>
#endif
#include <trantor/utils/Date.h>
#include <trantor/utils/Logger.h>
#include <json/json.h>
#include <string>
#include <string_view>
#include <memory>
#include <vector>
#include <tuple>
#include <stdint.h>
#include <iostream>

namespace drogon
{
namespace orm
{
class DbClient;
using DbClientPtr = std::shared_ptr<DbClient>;
}
}
namespace drogon_model
{
namespace amr_01
{
namespace amr_ros2
{

class Straightlink
{
  public:
    struct Cols
    {
        static const std::string _id_straight_link;
        static const std::string _id_start;
        static const std::string _id_end;
        static const std::string _map_id;
        static const std::string _max_velocity;
    };

    static const int primaryKeyNumber;
    static const std::string tableName;
    static const bool hasPrimaryKey;
    static const std::string primaryKeyName;
    using PrimaryKeyType = int32_t;
    const PrimaryKeyType &getPrimaryKey() const;

    /**
     * @brief constructor
     * @param r One row of records in the SQL query result.
     * @param indexOffset Set the offset to -1 to access all columns by column names,
     * otherwise access all columns by offsets.
     * @note If the SQL is not a style of 'select * from table_name ...' (select all
     * columns by an asterisk), please set the offset to -1.
     */
    explicit Straightlink(const drogon::orm::Row &r, const ssize_t indexOffset = 0) noexcept;

    /**
     * @brief constructor
     * @param pJson The json object to construct a new instance.
     */
    explicit Straightlink(const Json::Value &pJson) noexcept(false);

    /**
     * @brief constructor
     * @param pJson The json object to construct a new instance.
     * @param pMasqueradingVector The aliases of table columns.
     */
    Straightlink(const Json::Value &pJson, const std::vector<std::string> &pMasqueradingVector) noexcept(false);

    Straightlink() = default;

    void updateByJson(const Json::Value &pJson) noexcept(false);
    void updateByMasqueradedJson(const Json::Value &pJson,
                                 const std::vector<std::string> &pMasqueradingVector) noexcept(false);
    static bool validateJsonForCreation(const Json::Value &pJson, std::string &err);
    static bool validateMasqueradedJsonForCreation(const Json::Value &,
                                                const std::vector<std::string> &pMasqueradingVector,
                                                    std::string &err);
    static bool validateJsonForUpdate(const Json::Value &pJson, std::string &err);
    static bool validateMasqueradedJsonForUpdate(const Json::Value &,
                                          const std::vector<std::string> &pMasqueradingVector,
                                          std::string &err);
    static bool validJsonOfField(size_t index,
                          const std::string &fieldName,
                          const Json::Value &pJson,
                          std::string &err,
                          bool isForCreation);

    /**  For column id_straight_link  */
    ///Get the value of the column id_straight_link, returns the default value if the column is null
    const int32_t &getValueOfIdStraightLink() const noexcept;
    ///Return a shared_ptr object pointing to the column const value, or an empty shared_ptr object if the column is null
    const std::shared_ptr<int32_t> &getIdStraightLink() const noexcept;
    ///Set the value of the column id_straight_link
    void setIdStraightLink(const int32_t &pIdStraightLink) noexcept;

    /**  For column id_start  */
    ///Get the value of the column id_start, returns the default value if the column is null
    const int32_t &getValueOfIdStart() const noexcept;
    ///Return a shared_ptr object pointing to the column const value, or an empty shared_ptr object if the column is null
    const std::shared_ptr<int32_t> &getIdStart() const noexcept;
    ///Set the value of the column id_start
    void setIdStart(const int32_t &pIdStart) noexcept;

    /**  For column id_end  */
    ///Get the value of the column id_end, returns the default value if the column is null
    const int32_t &getValueOfIdEnd() const noexcept;
    ///Return a shared_ptr object pointing to the column const value, or an empty shared_ptr object if the column is null
    const std::shared_ptr<int32_t> &getIdEnd() const noexcept;
    ///Set the value of the column id_end
    void setIdEnd(const int32_t &pIdEnd) noexcept;

    /**  For column map_id  */
    ///Get the value of the column map_id, returns the default value if the column is null
    const int32_t &getValueOfMapId() const noexcept;
    ///Return a shared_ptr object pointing to the column const value, or an empty shared_ptr object if the column is null
    const std::shared_ptr<int32_t> &getMapId() const noexcept;
    ///Set the value of the column map_id
    void setMapId(const int32_t &pMapId) noexcept;

    /**  For column max_velocity  */
    ///Get the value of the column max_velocity, returns the default value if the column is null
    const double &getValueOfMaxVelocity() const noexcept;
    ///Return a shared_ptr object pointing to the column const value, or an empty shared_ptr object if the column is null
    const std::shared_ptr<double> &getMaxVelocity() const noexcept;
    ///Set the value of the column max_velocity
    void setMaxVelocity(const double &pMaxVelocity) noexcept;
    void setMaxVelocityToNull() noexcept;


    static size_t getColumnNumber() noexcept {  return 5;  }
    static const std::string &getColumnName(size_t index) noexcept(false);

    Json::Value toJson() const;
    Json::Value toMasqueradedJson(const std::vector<std::string> &pMasqueradingVector) const;
    /// Relationship interfaces
  private:
    friend drogon::orm::Mapper<Straightlink>;
    friend drogon::orm::BaseBuilder<Straightlink, true, true>;
    friend drogon::orm::BaseBuilder<Straightlink, true, false>;
    friend drogon::orm::BaseBuilder<Straightlink, false, true>;
    friend drogon::orm::BaseBuilder<Straightlink, false, false>;
#ifdef __cpp_impl_coroutine
    friend drogon::orm::CoroMapper<Straightlink>;
#endif
    static const std::vector<std::string> &insertColumns() noexcept;
    void outputArgs(drogon::orm::internal::SqlBinder &binder) const;
    const std::vector<std::string> updateColumns() const;
    void updateArgs(drogon::orm::internal::SqlBinder &binder) const;
    ///For mysql or sqlite3
    void updateId(const uint64_t id);
    std::shared_ptr<int32_t> idStraightLink_;
    std::shared_ptr<int32_t> idStart_;
    std::shared_ptr<int32_t> idEnd_;
    std::shared_ptr<int32_t> mapId_;
    std::shared_ptr<double> maxVelocity_;
    struct MetaData
    {
        const std::string colName_;
        const std::string colType_;
        const std::string colDatabaseType_;
        const ssize_t colLength_;
        const bool isAutoVal_;
        const bool isPrimaryKey_;
        const bool notNull_;
    };
    static const std::vector<MetaData> metaData_;
    bool dirtyFlag_[5]={ false };
  public:
    static const std::string &sqlForFindingByPrimaryKey()
    {
        static const std::string sql="select * from " + tableName + " where id_straight_link = $1";
        return sql;
    }

    static const std::string &sqlForDeletingByPrimaryKey()
    {
        static const std::string sql="delete from " + tableName + " where id_straight_link = $1";
        return sql;
    }
    std::string sqlForInserting(bool &needSelection) const
    {
        std::string sql="insert into " + tableName + " (";
        size_t parametersCount = 0;
        needSelection = false;
            sql += "id_straight_link,";
            ++parametersCount;
        if(dirtyFlag_[1])
        {
            sql += "id_start,";
            ++parametersCount;
        }
        if(dirtyFlag_[2])
        {
            sql += "id_end,";
            ++parametersCount;
        }
        if(dirtyFlag_[3])
        {
            sql += "map_id,";
            ++parametersCount;
        }
        if(dirtyFlag_[4])
        {
            sql += "max_velocity,";
            ++parametersCount;
        }
        needSelection=true;
        if(parametersCount > 0)
        {
            sql[sql.length()-1]=')';
            sql += " values (";
        }
        else
            sql += ") values (";

        int placeholder=1;
        char placeholderStr[64];
        size_t n=0;
        sql +="default,";
        if(dirtyFlag_[1])
        {
            n = snprintf(placeholderStr,sizeof(placeholderStr),"$%d,",placeholder++);
            sql.append(placeholderStr, n);
        }
        if(dirtyFlag_[2])
        {
            n = snprintf(placeholderStr,sizeof(placeholderStr),"$%d,",placeholder++);
            sql.append(placeholderStr, n);
        }
        if(dirtyFlag_[3])
        {
            n = snprintf(placeholderStr,sizeof(placeholderStr),"$%d,",placeholder++);
            sql.append(placeholderStr, n);
        }
        if(dirtyFlag_[4])
        {
            n = snprintf(placeholderStr,sizeof(placeholderStr),"$%d,",placeholder++);
            sql.append(placeholderStr, n);
        }
        if(parametersCount > 0)
        {
            sql.resize(sql.length() - 1);
        }
        if(needSelection)
        {
            sql.append(") returning *");
        }
        else
        {
            sql.append(1, ')');
        }
        LOG_TRACE << sql;
        return sql;
    }
};
} // namespace amr_ros2
} // namespace amr_01
} // namespace drogon_model
