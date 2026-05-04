/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "PriorityTerrainMeshLayer"
#include "TerrainEngineNode"

using namespace osgEarth;

#define LC "[PriorityTerrainMeshLayer] "

REGISTER_OSGEARTH_LAYER(priorityterrainmesh, PriorityTerrainMeshLayer);

//...........................................................................

void
PriorityTerrainMeshLayer::Options::fromConfig(const Config& conf)
{
    for (const auto& c : conf.child("layers").children())
        _layers.push_back(ConfigOptions(c));
}

Config
PriorityTerrainMeshLayer::Options::getConfig() const
{
    Config conf = super::getConfig();
    if (!_layers.empty())
    {
        Config layersConf("layers");
        for (const auto& o : _layers)
            layersConf.add(o.getConfig());
        conf.set(layersConf);
    }
    return conf;
}

//...........................................................................

void
PriorityTerrainMeshLayer::addLayer(TerrainMeshLayer* layer)
{
    if (isOpen())
    {
        OE_WARN << LC << "addLayer() called after open — ignored" << std::endl;
        return;
    }
    if (layer)
        _layers.push_back(layer);
}

void
PriorityTerrainMeshLayer::insertLayer(unsigned index, TerrainMeshLayer* layer)
{
    if (isOpen())
    {
        OE_WARN << LC << "insertLayer() called after open — ignored" << std::endl;
        return;
    }
    if (layer)
    {
        const auto pos = _layers.begin() +
            std::min(static_cast<std::size_t>(index), _layers.size());
        _layers.insert(pos, layer);
    }
}

Status
PriorityTerrainMeshLayer::openImplementation()
{
    OE_RETURN_STATUS_ON_ERROR(super::openImplementation());

    // If no layers were added programmatically, restore from serialized options.
    if (_layers.empty())
    {
        for (const auto& conf : options().layers())
        {
            osg::ref_ptr<Layer> base = Layer::create(conf);
            auto* ml = dynamic_cast<TerrainMeshLayer*>(base.get());
            if (ml)
                _layers.push_back(ml);
            else
                OE_WARN << LC << "Serialized child is not a TerrainMeshLayer — discarded" << std::endl;
        }
    }

    // Open each child. Partial failures are tolerated (fallback still works with remaining layers).
    for (auto& layer : _layers)
    {
        const Status s = layer->open(getReadOptions());
        if (s.isError())
        {
            OE_WARN << LC << "Child layer \"" << layer->getName()
                    << "\" failed to open: " << s.message() << std::endl;
        }
    }

    return Status::NoError;
}

Status
PriorityTerrainMeshLayer::closeImplementation()
{
    for (auto& layer : _layers)
        layer->close();
    return super::closeImplementation();
}

void
PriorityTerrainMeshLayer::addedToMap(const Map* map)
{
    super::addedToMap(map);

    osg::ref_ptr<const Profile> profile = getProfile();
    DataExtentList outputExtents;
    bool allExtentsKnown = true;

    for (auto& layer : _layers)
    {
        if (layer->isOpen())
        {
            layer->addedToMap(map);

            if (profile)
            {
                DataExtentList childExtents;
                layer->getDataExtents(childExtents);

                if (childExtents.empty())
                {
                    allExtentsKnown = false;
                }
                else
                {
                    for (const auto& de : childExtents)
                    {
                        GeoExtent ext = de.transform(profile->getSRS());
                        if (ext.isValid())
                        {
                            const unsigned minL = de.minLevel().isSet() ? *de.minLevel() : 0u;
                            const unsigned maxL = de.maxLevel().isSet()
                                ? profile->getEquivalentLOD(layer->getProfile(), *de.maxLevel())
                                : 23u;
                            outputExtents.emplace_back(ext, minL, maxL);
                        }
                    }
                }
            }
        }
    }

    if (allExtentsKnown && !outputExtents.empty())
        setDataExtents(outputExtents);
}

void
PriorityTerrainMeshLayer::removedFromMap(const Map* map)
{
    for (auto& layer : _layers)
        if (layer->isOpen())
            layer->removedFromMap(map);
    super::removedFromMap(map);
}

void
PriorityTerrainMeshLayer::prepareForRendering(TerrainEngine* engine)
{
    super::prepareForRendering(engine);
    for (auto& layer : _layers)
        if (layer->isOpen())
            layer->prepareForRendering(engine);
}

TileMesh
PriorityTerrainMeshLayer::createTileImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
    for (const auto& layer : _layers)
    {
        if (!layer->isOpen())
            continue;
        TileMesh mesh = layer->createTile(key, progress);
        if (mesh.verts.valid() && !mesh.verts->empty())
        {
            applyConstraints(key, mesh);
            return mesh;
        }
    }
    return TileMesh{};
}
