#ifndef PRESETMANAGER_H
#define PRESETMANAGER_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVariantMap>

/**
 * @brief PresetManager
 * Persists named PID/Reference/Config snapshots as JSON in the app data directory.
 * Exposed to QML via the `presetManager` context property.
 *
 * On-disk layout (QStandardPaths::AppDataLocation/pid_presets.json):
 * [
 *   { "name": "hover-v1", "savedAt": "2026-04-16T…Z",
 *     "values": { "attKpX": 0.8, "attKpY": 0.8, … } },
 *   …
 * ]
 */
class PresetManager : public QObject {
    Q_OBJECT

public:
    explicit PresetManager(QObject* parent = nullptr);

    Q_PROPERTY(QStringList presets READ presets NOTIFY presetsChanged)
    Q_PROPERTY(QString storagePath READ storagePath CONSTANT)

    QStringList presets() const { return m_names; }
    QString     storagePath() const;

    /// Save (or overwrite) a preset with the given values. Empty name → false.
    Q_INVOKABLE bool save(const QString& name, const QVariantMap& values);

    /// Load values for a preset; returns an empty map if not found.
    Q_INVOKABLE QVariantMap load(const QString& name) const;

    /// Remove a preset; no-op if not found.
    Q_INVOKABLE bool remove(const QString& name);

    /// Rename an existing preset. Fails if `from` doesn't exist or `to` already exists.
    Q_INVOKABLE bool rename(const QString& from, const QString& to);

signals:
    void presetsChanged();

private:
    void loadFromDisk();
    bool writeToDisk() const;

    struct Preset {
        QString      name;
        QString      savedAt;
        QVariantMap  values;
    };

    QList<Preset> m_entries;
    QStringList   m_names;  // cached for the QML binding

    void refreshNames();
};

#endif // PRESETMANAGER_H
