#include "PresetManager.h"

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStandardPaths>

PresetManager::PresetManager(QObject* parent)
    : QObject(parent)
{
    loadFromDisk();
}

QString PresetManager::storagePath() const
{
#ifdef ULYSSES_PRESETS_PATH
    // Injected at configure time; points at embedded-software/ground-station/presets/pid_presets.json
    // so every team member reads/writes the same file tracked in git.
    return QStringLiteral(ULYSSES_PRESETS_PATH);
#else
    const QString dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    return dir + QStringLiteral("/pid_presets.json");
#endif
}

void PresetManager::refreshNames()
{
    m_names.clear();
    for (const auto& p : m_entries)
        m_names << p.name;
}

void PresetManager::loadFromDisk()
{
    m_entries.clear();
    const QString path = storagePath();
    QFile f(path);
    if (!f.exists()) {
        refreshNames();
        emit presetsChanged();
        return;
    }
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "PresetManager: cannot read" << path << f.errorString();
        refreshNames();
        emit presetsChanged();
        return;
    }

    QJsonParseError err{};
    const auto doc = QJsonDocument::fromJson(f.readAll(), &err);
    f.close();
    if (err.error != QJsonParseError::NoError || !doc.isArray()) {
        qWarning() << "PresetManager: invalid JSON in" << path << err.errorString();
        refreshNames();
        emit presetsChanged();
        return;
    }

    for (const QJsonValue& v : doc.array()) {
        if (!v.isObject()) continue;
        const auto obj = v.toObject();
        Preset p;
        p.name    = obj.value(QStringLiteral("name")).toString();
        p.savedAt = obj.value(QStringLiteral("savedAt")).toString();
        p.values  = obj.value(QStringLiteral("values")).toObject().toVariantMap();
        if (!p.name.isEmpty())
            m_entries.push_back(std::move(p));
    }
    refreshNames();
    emit presetsChanged();
}

bool PresetManager::writeToDisk() const
{
    const QString path = storagePath();
    QFileInfo info(path);
    QDir dir = info.absoluteDir();
    if (!dir.exists() && !dir.mkpath(QStringLiteral("."))) {
        qWarning() << "PresetManager: cannot create dir" << dir.absolutePath();
        return false;
    }

    QJsonArray arr;
    for (const auto& p : m_entries) {
        QJsonObject obj;
        obj[QStringLiteral("name")]    = p.name;
        obj[QStringLiteral("savedAt")] = p.savedAt;
        obj[QStringLiteral("values")]  = QJsonObject::fromVariantMap(p.values);
        arr.push_back(obj);
    }

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        qWarning() << "PresetManager: cannot write" << path << f.errorString();
        return false;
    }
    f.write(QJsonDocument(arr).toJson(QJsonDocument::Indented));
    f.close();
    return true;
}

bool PresetManager::save(const QString& name, const QVariantMap& values)
{
    const QString trimmed = name.trimmed();
    if (trimmed.isEmpty())
        return false;

    Preset p{ trimmed, QDateTime::currentDateTimeUtc().toString(Qt::ISODate), values };

    bool replaced = false;
    for (auto& e : m_entries) {
        if (e.name == trimmed) {
            e = p;
            replaced = true;
            break;
        }
    }
    if (!replaced)
        m_entries.push_back(std::move(p));

    if (!writeToDisk())
        return false;

    refreshNames();
    emit presetsChanged();
    return true;
}

QVariantMap PresetManager::load(const QString& name) const
{
    for (const auto& e : m_entries) {
        if (e.name == name)
            return e.values;
    }
    return {};
}

bool PresetManager::remove(const QString& name)
{
    const int before = m_entries.size();
    m_entries.erase(std::remove_if(m_entries.begin(), m_entries.end(),
                                   [&](const Preset& p){ return p.name == name; }),
                    m_entries.end());
    if (m_entries.size() == before)
        return false;

    if (!writeToDisk())
        return false;

    refreshNames();
    emit presetsChanged();
    return true;
}

bool PresetManager::rename(const QString& from, const QString& to)
{
    const QString target = to.trimmed();
    if (from.isEmpty() || target.isEmpty() || from == target)
        return false;

    // Target must not collide with an existing preset.
    for (const auto& e : m_entries) {
        if (e.name == target)
            return false;
    }
    for (auto& e : m_entries) {
        if (e.name == from) {
            e.name = target;
            if (!writeToDisk())
                return false;
            refreshNames();
            emit presetsChanged();
            return true;
        }
    }
    return false;
}
