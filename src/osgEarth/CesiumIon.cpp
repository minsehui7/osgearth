/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "CesiumIon"
#include <osgEarth/Registry>
#include <osgEarth/JsonUtils>
#include <osgEarth/TDTiles>
#include <osgEarth/TMS>
#include <osgEarth/Bing>
#include <osgEarth/Notify>
#include <osgEarth/Locators>
#include <osgEarth/GeoData>
#include <osgEarth/Progress>

#include <osgUtil/SmoothingVisitor>
#include <atomic>
#include <chrono>

using namespace osgEarth;
using namespace osgEarth::Contrib::ThreeDTiles;

#undef LC
#define LC "[CesiumIon] "

//............................................................................

Status
CesiumIonResource::open(const URI& server,
                        const std::string& assetId,
                        const std::string& token,
                        const osgDB::Options* readOptions)
{
    if (assetId.empty())
    {
        return Status::Error(Status::ConfigurationError, "Fail: driver requires a valid \"asset_id\" property");
    }

    if (token.empty())
    {
        return Status::Error(Status::ConfigurationError, "Fail: driver requires a valid \"token\" property");
    }

    std::stringstream buf;
    buf << server.full();
    if (!endsWith(server.full(), "/")) buf << "/";
    buf << "v1/assets/" << assetId << "/endpoint?access_token=" << token;
    URI endpoint(buf.str());
    OE_DEBUG << "Getting endpoint " << endpoint.full() << std::endl;

    ReadResult r = URI(endpoint).readString(readOptions);
    if (r.failed())
        return Status::Error(Status::ConfigurationError, "Failed to get metadata from asset endpoint");

    Json::Value doc;
    Json::Reader reader;
    if (!reader.parse(r.getString(), doc))
    {
        return Status::Error(Status::ConfigurationError, "Failed to parse metadata from asset endpoint");
    }

    _resourceUrl = doc["url"].asString();
    _resourceToken = doc["accessToken"].asString();

    // Configure the accept header
    std::stringstream buf2;
    //buf2 << "*/*;access_token=" << _resourceToken;
    buf2 << "Bearer " << _resourceToken;
    _acceptHeader = buf2.str();

    if (doc.isMember("externalType"))
    {
        _externalType = doc["externalType"].asString();
        if (doc.isMember("options"))
        {
            _externalOptions = doc["options"];
        }
    }

    return STATUS_OK;
}

//........................................................................

Config
CesiumIonImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
            conf.set("server", _server);
            conf.set("asset_id", _assetId);
            conf.set("token", _token);
    return conf;
}

void
CesiumIonImageLayer::Options::fromConfig(const Config& conf)
{
    _server.init("https://api.cesium.com/");
    conf.get("server", _server);
    conf.get("asset_id", _assetId);
    conf.get("token", _token);
}

Config
CesiumIonImageLayer::Options::getMetadata()
{
    return Config::readJSON(R"( {
        "name" : "CesiumIon Service",
        "properties" : [
        ]
    } )");
}

REGISTER_OSGEARTH_LAYER(cesiumionimage, CesiumIonImageLayer);

OE_LAYER_PROPERTY_IMPL(CesiumIonImageLayer, URI, Server, server);
OE_LAYER_PROPERTY_IMPL(CesiumIonImageLayer, std::string, AssetId, assetId);
OE_LAYER_PROPERTY_IMPL(CesiumIonImageLayer, std::string, Token, token);

void
CesiumIonImageLayer::init()
{
    ImageLayer::init();
}

Status
CesiumIonImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    const char* key = ::getenv("OSGEARTH_CESIUMION_KEY");
    if (key)
        _key = key;
    else
        _key = options().token().get();

    if (_key.empty())
    {
        return Status(Status::ConfigurationError, "CesiumIon API key is required");
    }

    CesiumIonResource ionResource;
    Status status = ionResource.open(
        options().server().get(),
        options().assetId().get(),
        _key,
        getReadOptions());

    if (status.isOK())
    {
        URIContext uriContext(ionResource._resourceUrl);
        uriContext.addHeader("authorization", ionResource._acceptHeader);
        URI tmsURI = URI("tilemapresource.xml", uriContext);


        if (ionResource._externalType.empty())
        {
            TMSImageLayer* tmsImageLayer = new TMSImageLayer();
            tmsImageLayer->setURL(tmsURI);
            _imageLayer = tmsImageLayer;
        }
        else
        {
            if (ionResource._externalType == "BING")
            {
                BingImageLayer *bingImageLayer = new BingImageLayer();
                bingImageLayer->setAPIKey(ionResource._externalOptions["key"].asString());
                bingImageLayer->setImagerySet(ionResource._externalOptions["mapStyle"].asString());
                _imageLayer = bingImageLayer;
            }
        }

        if (_imageLayer)
        {
            _imageLayer->setName(getName());
            status = _imageLayer->open();
            if (status.isError())
                return status;
            setProfile(_imageLayer->getProfile());
            DataExtentList dataExtents;
            _imageLayer->getDataExtents(dataExtents);
            setDataExtents(dataExtents);
        }
        else
        {
            return Status::Error("Unsupported Cesium Ion image layer");
        }
    }

    return status;
}

Status
CesiumIonImageLayer::closeImplementation()
{
    if (_imageLayer.valid())
    {
        _imageLayer->close();
        _imageLayer = NULL;
    }
    return ImageLayer::closeImplementation();
}

GeoImage
CesiumIonImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (_imageLayer)
    {
        return _imageLayer->createImage(key, progress);
    }
    return GeoImage(Status("Invalid image layer"));    
}

//........................................................................

Config
CesiumIon3DTilesLayer::Options::getConfig() const
{
    Config conf = ThreeDTilesLayer::Options::getConfig();
    conf.set("server", _server);
    conf.set("asset_id", _assetId);
    conf.set("token", _token);
    return conf;
}

void
CesiumIon3DTilesLayer::Options::fromConfig(const Config& conf)
{
    _server.init("https://api.cesium.com/");
    conf.get("server", _server);
    conf.get("asset_id", _assetId);
    conf.get("token", _token);
}

void
CesiumIon3DTilesLayer::init()
{
    ThreeDTilesLayer::init();
}

Status
CesiumIon3DTilesLayer::openImplementation()
{
    const char* key = ::getenv("OSGEARTH_CESIUMION_KEY");
    if (key)
        _key = key;
    else
        _key = options().token().get();

    if (_key.empty())
    {
        return Status(Status::ConfigurationError, "CesiumIon API key is required");
    }

    CesiumIonResource ionResource;
    Status status = ionResource.open(
        options().server().get(),
        options().assetId().get(),
        _key,
        getReadOptions());

    URI serverURI;
    if (status.isOK())
    {
        URIContext uriContext;
        uriContext.addHeader("authorization", ionResource._acceptHeader);
        serverURI = URI(ionResource._resourceUrl, uriContext);
    }

    Status parentStatus = VisibleLayer::openImplementation();
    if (parentStatus.isError())
        return parentStatus;

    ReadResult rr = serverURI.readString();
    if (rr.failed())
    {
        return Status(Status::ResourceUnavailable, Stringify() << "Error loading tileset: " << rr.errorDetail());
    }

    //OE_NOTICE << "Read tileset " << rr.getString() << std::endl;

    Tileset* tileset = Tileset::create(rr.getString(), serverURI.full());
    if (!tileset)
    {
        return Status(Status::GeneralError, "Bad tileset");
    }

    // Clone the read options
    osg::ref_ptr< osgDB::Options > readOptions = osgEarth::Registry::instance()->cloneOrCreateOptions(this->getReadOptions());

    _tilesetNode = new ThreeDTilesetNode(tileset, ionResource._acceptHeader, getSceneGraphCallbacks(), readOptions.get());
    _tilesetNode->setMaximumScreenSpaceError(*options().maximumScreenSpaceError());

    return STATUS_OK;
}

REGISTER_OSGEARTH_LAYER(cesiumion3dtiles, CesiumIon3DTilesLayer);
OE_LAYER_PROPERTY_IMPL(CesiumIon3DTilesLayer, URI, Server, server);
OE_LAYER_PROPERTY_IMPL(CesiumIon3DTilesLayer, std::string, AssetId, assetId);
OE_LAYER_PROPERTY_IMPL(CesiumIon3DTilesLayer, std::string, Token, token);





//........................................................................

Config
CesiumIonTerrainMeshLayer::Options::getConfig() const
{
    Config conf = TerrainMeshLayer::Options::getConfig();
    conf.set("server", _server);
    conf.set("asset_id", _assetId);
    conf.set("token", _token);
    return conf;
}

void
CesiumIonTerrainMeshLayer::Options::fromConfig(const Config& conf)
{
    _server.init("https://api.cesium.com/");
    conf.get("server", _server);
    conf.get("asset_id", _assetId);
    conf.get("token", _token);
}

Config
CesiumIonTerrainMeshLayer::Options::getMetadata()
{
    return Config::readJSON(R"(
        { "name" : "CesiumIon Service",
            "properties" : [
            ]
        })");    
}

REGISTER_OSGEARTH_LAYER(cesiumionterrainmesh, CesiumIonTerrainMeshLayer);

OE_LAYER_PROPERTY_IMPL(CesiumIonTerrainMeshLayer, URI, Server, server);
OE_LAYER_PROPERTY_IMPL(CesiumIonTerrainMeshLayer, std::string, AssetId, assetId);
OE_LAYER_PROPERTY_IMPL(CesiumIonTerrainMeshLayer, std::string, Token, token);

void
CesiumIonTerrainMeshLayer::init()
{
    TerrainMeshLayer::init();
}

Status
CesiumIonTerrainMeshLayer::openImplementation()
{
    Status parent = TerrainMeshLayer::openImplementation();
    if (parent.isError())
        return parent;

    const char* key = ::getenv("OSGEARTH_CESIUMION_KEY");
    if (key)
        _key = key;
    else
        _key = options().token().get();

    // Direct URL mode: no token → treat server URL as the quantized-mesh tile base URI.
    // This supports local servers (e.g. cesium-terrain-server at http://127.0.0.1:8080/).
    // Tile URL pattern: {server}/{z}/{x}/{y}.terrain
    if (_key.empty())
    {
        if (!options().server().isSet() || options().server()->empty())
            return Status(Status::ConfigurationError,
                          "CesiumIonTerrainMeshLayer requires either a token (Cesium Ion) "
                          "or a server URL (direct quantized-mesh server, no token)");
        _assetURI = options().server().get();
        return Status::NoError;
    }

    CesiumIonResource ionResource;
    Status status = ionResource.open(
        options().server().get(),
        options().assetId().get(),
        _key,
        getReadOptions());

    if (status.isOK())
    {
        URIContext uriContext;
        uriContext.addHeader("authorization", ionResource._acceptHeader);
        _assetURI = URI(ionResource._resourceUrl, uriContext);
    }

    return status;
}

Status
CesiumIonTerrainMeshLayer::closeImplementation()
{
    return TerrainMeshLayer::closeImplementation();
}


///////

namespace
{
    std::atomic<bool> s_logTerrainHttpResponses{ false };

    const char* readResultCodeName(ReadResult::Code code)
    {
        switch (code)
        {
        case ReadResult::RESULT_OK: return "OK";
        case ReadResult::RESULT_CANCELED: return "CANCELED";
        case ReadResult::RESULT_UNAUTHORIZED: return "UNAUTHORIZED";
        case ReadResult::RESULT_NOT_FOUND: return "NOT_FOUND";
        case ReadResult::RESULT_EXPIRED: return "EXPIRED";
        case ReadResult::RESULT_SERVER_ERROR: return "SERVER_ERROR";
        case ReadResult::RESULT_TIMEOUT: return "TIMEOUT";
        case ReadResult::RESULT_NO_READER: return "NO_READER";
        case ReadResult::RESULT_READER_ERROR: return "READER_ERROR";
        case ReadResult::RESULT_UNKNOWN_ERROR: return "UNKNOWN_ERROR";
        case ReadResult::RESULT_NOT_IMPLEMENTED: return "NOT_IMPLEMENTED";
        case ReadResult::RESULT_NOT_MODIFIED: return "NOT_MODIFIED";
        default: return "CLIENT_ERROR";
        }
    }

    struct QuantizedMeshHeader
    {
        // The center of the tile in Earth-centered Fixed coordinates.
        double CenterX;
        double CenterY;
        double CenterZ;

        // The minimum and maximum heights in the area covered by this tile.
        // The minimum may be lower and the maximum may be higher than
        // the height of any vertex in this tile in the case that the min/max vertex
        // was removed during mesh simplification, but these are the appropriate
        // values to use for analysis or visualization.
        float MinimumHeight;
        float MaximumHeight;

        // The tile's bounding sphere.  The X,Y,Z coordinates are again expressed
        // in Earth-centered Fixed coordinates, and the radius is in meters.
        double BoundingSphereCenterX;
        double BoundingSphereCenterY;
        double BoundingSphereCenterZ;
        double BoundingSphereRadius;

        // The horizon occlusion point, expressed in the ellipsoid-scaled Earth-centered Fixed frame.
        // If this point is below the horizon, the entire tile is below the horizon.
        // See http://cesiumjs.org/2013/04/25/Horizon-culling/ for more information.
        double HorizonOcclusionPointX;
        double HorizonOcclusionPointY;
        double HorizonOcclusionPointZ;
    };

    struct VertexData
    {
        unsigned int vertexCount;
        std::vector<unsigned short> u;
        std::vector<unsigned short> v;
        std::vector<unsigned short> height;
    };

    int zig_zag_decode(int n)
    {
        return (n >> 1) ^ (-(n & 1));
    }


    TileMesh quantizedMeshToTileMesh(const osgEarth::TileKey& key, std::stringstream& buf)
    {
        // Parse the header
        QuantizedMeshHeader header;
        buf.read(reinterpret_cast<char*>(&header), sizeof(QuantizedMeshHeader));

        VertexData vertexData;
        buf.read(reinterpret_cast<char*>(&vertexData.vertexCount), sizeof(unsigned int));
        vertexData.u.resize(vertexData.vertexCount);
        vertexData.v.resize(vertexData.vertexCount);
        vertexData.height.resize(vertexData.vertexCount);
        buf.read(reinterpret_cast<char*>(vertexData.u.data()), sizeof(unsigned short) * vertexData.vertexCount);
        buf.read(reinterpret_cast<char*>(vertexData.v.data()), sizeof(unsigned short) * vertexData.vertexCount);
        buf.read(reinterpret_cast<char*>(vertexData.height.data()), sizeof(unsigned short) * vertexData.vertexCount);

        unsigned short u = 0;
        unsigned short v = 0;
        unsigned short height = 0;
        for (unsigned int i = 0; i < vertexData.vertexCount; ++i)
        {
            u += zig_zag_decode(vertexData.u[i]);
            v += zig_zag_decode(vertexData.v[i]);
            height += zig_zag_decode(vertexData.height[i]);
            vertexData.u[i] = u;
            vertexData.v[i] = v;
            vertexData.height[i] = height;
        }

        unsigned int triangleCount;
        buf.read(reinterpret_cast<char*>(&triangleCount), sizeof(unsigned int));

        osg::ref_ptr< osg::DrawElements > drawElements;

        if (vertexData.vertexCount > 65536)
        {
            std::vector<unsigned int> indices;
            indices.resize(triangleCount * 3u);
            buf.read(reinterpret_cast<char*>(indices.data()), sizeof(unsigned int) * indices.size());

            unsigned int highest = 0;
            for (unsigned int i = 0; i < indices.size(); ++i)
            {
                unsigned int code = indices[i];
                indices[i] = highest - code;
                if (code == 0) {
                    ++highest;
                }
            }

            drawElements = new osg::DrawElementsUInt(GL_TRIANGLES);
            for (unsigned int i = 0; i < indices.size(); ++i)
            {
                drawElements->addElement(indices[i]);
            }
        }
        else
        {
            std::vector<unsigned short> indices;
            indices.resize(triangleCount * 3u);
            buf.read(reinterpret_cast<char*>(indices.data()), sizeof(unsigned short) * indices.size());

            unsigned short highest = 0;
            for (unsigned int i = 0; i < indices.size(); ++i)
            {
                unsigned short code = indices[i];
                indices[i] = highest - code;
                if (code == 0) {
                    ++highest;
                }
            }

            drawElements = new osg::DrawElementsUShort(GL_TRIANGLES);
            for (unsigned int i = 0; i < indices.size(); ++i)
            {
                drawElements->addElement(indices[i]);
            }
        }

        TileMesh mesh;
        osgEarth::GeoLocator locator(key.getExtent());
        osgEarth::GeoPoint centroid_world = key.getExtent().getCentroid();
        // LOD 18+ 에서 float 행렬(osg::Matrix)은 ECEF 절댓값(~6,370km) 대비
        // 타일 크기(~1.5km)의 비율로 재앙적 소거가 발생하므로 Matrixd를 사용한다.
        osg::Matrixd world2local, local2world;
        centroid_world.createWorldToLocal(world2local);
        local2world.invert(world2local);

        osg::ref_ptr<osg::VertexBufferObject> vbo = new osg::VertexBufferObject();

        osg::ref_ptr< osg::Vec3Array > verts = new osg::Vec3Array(vertexData.vertexCount);
        verts->setVertexBufferObject(vbo.get());
        verts->setBinding(osg::Array::BIND_PER_VERTEX);

        osg::ref_ptr< osg::Vec3Array > texCoords = new osg::Vec3Array(vertexData.vertexCount);
        texCoords->setVertexBufferObject(vbo.get());
        texCoords->setBinding(osg::Array::BIND_PER_VERTEX);

        osg::Vec3d unit;
        osg::Vec3d model;
        osg::Vec3d modelLTP;

        for (unsigned int i = 0; i < vertexData.vertexCount; ++i)
        {
            float tileWith = header.MaximumHeight - header.MinimumHeight;
            float s = (float)vertexData.u[i] / 32767.0f;
            float t = (float)vertexData.v[i] / 32767.0f;
            float r = (float)vertexData.height[i] / 32767.0f;

            float height = header.MinimumHeight + r * (header.MaximumHeight - header.MinimumHeight);
            unit.set(s, t, height);
            locator.unitToWorld(unit, model);
            // Matrixd로 변환 → LOD 18+ 버텍스 정밀도 손실 방지
            modelLTP = model * world2local;

            int default_marker = VERTEX_VISIBLE | VERTEX_CONSTRAINT | VERTEX_HAS_ELEVATION;
            (*texCoords)[i].set(osg::Vec3(s, t, (float)default_marker));
            (*verts)[i].set(modelLTP);

            // Request oct encoded normals?
        }

        osg::ref_ptr< osg::Vec3Array > normals;
        if (!normals)
        {
            osg::ref_ptr< osg::Geode > geode = new osg::Geode;
            osg::ref_ptr< osg::Geometry > geometry = new osg::Geometry;
            geometry->setVertexArray(verts);
            geometry->addPrimitiveSet(drawElements);
            geode->addDrawable(geometry);
            osgUtil::SmoothingVisitor sv;
            geode->accept(sv);
            normals = static_cast<osg::Vec3Array*>(geometry->getNormalArray());
            if (normals)
            {
                normals->setVertexBufferObject(vbo.get());
            }
        }


        mesh.verts = verts;
        mesh.uvs = texCoords;
        mesh.normals = normals;
        mesh.indices = drawElements;
        mesh.localToWorld = local2world;

        return mesh;
    }

    static URI makeCesiumTerrainTileUri(const TileKey& key, const URI& assetBase)
    {
        unsigned tx, ty;
        key.getTileXY(tx, ty);
        unsigned int numRows = 0, numCols = 0;
        key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
        ty = numRows - ty - 1u;
        return URI(
            Stringify() << assetBase.full() << key.getLevelOfDetail() << "/" << tx << "/" << ty << ".terrain",
            assetBase.context());
    }

    static TileMesh tryReadCesiumQuantizedMeshForKey(
        const TileKey& key,
        const URI& assetBase,
        const osgDB::Options* readOptions,
        ProgressCallback* progress,
        ReadResult::Code* out_code = nullptr,
        bool fastFail = false)
    {
        const URI tileURI = makeCesiumTerrainTileUri(key, assetBase);
        // fastFail 모드(폴백 레이어가 있는 경우): 1회만 시도.
        // 서버 다운/타임아웃 시 재시도 없이 즉시 반환해 폴백으로 넘어가도록 한다.
        const unsigned maxAttempts = fastFail ? 1u : 4u;
        ReadResult::Code lastCode = ReadResult::RESULT_NOT_FOUND;
        for (unsigned int attempt = 0; attempt < maxAttempts; ++attempt)
        {
            if (progress && progress->isCanceled())
            {
                lastCode = ReadResult::RESULT_CANCELED;
                break;
            }
            const ReadResult result = tileURI.readString(readOptions, progress);
            lastCode = result.code();
            if (s_logTerrainHttpResponses.load(std::memory_order_relaxed))
            {
                unsigned logX = 0u;
                unsigned logY = 0u;
                key.getTileXY(logX, logY);
                std::stringstream log;
                log << LC << "Terrain HTTP "
                    << (result.succeeded() ? "OK" : "FAIL")
                    << " code=" << readResultCodeName(result.code())
                    << " attempt=" << (attempt + 1u) << "/" << maxAttempts
                    << " lod=" << key.getLevelOfDetail()
                    << " x=" << logX
                    << " y=" << logY
                    << " bytes=" << result.getString().size()
                    << " url=" << tileURI.full();
                if (!result.errorDetail().empty())
                    log << " detail=" << result.errorDetail();
                OE_NOTICE << log.str() << std::endl;
            }
            if (result.succeeded())
            {
                if (out_code) *out_code = ReadResult::RESULT_OK;
                std::stringstream buf(result.getString());
                return quantizedMeshToTileMesh(key, buf);
            }
        }
        if (out_code) *out_code = lastCode;
        return TileMesh();
    }

    /// When a child .terrain is missing, reuse a loaded parent quantized mesh clipped to the child
    /// extent so elevation does not collapse to the ellipsoid grid fallback (GeometryPool).
    static TileMesh clipParentTerrainMeshToChildTile(const TileKey& childKey, const TileMesh& parentMesh)
    {
        TileMesh out;
        if (!parentMesh.verts.valid() || !parentMesh.indices.valid() || !parentMesh.uvs.valid())
            return out;

        const GeoExtent childEx = childKey.getExtent();
        if (!childEx.isValid())
            return out;

        const SpatialReference* horiz = childEx.getSRS();
        if (!horiz)
            return out;

        GeoLocator childLocator(childEx);
        const GeoPoint childCentroid = childEx.getCentroid();
        osg::Matrixd childW2L, childL2W;
        childCentroid.createWorldToLocal(childW2L);
        childL2W.invert(childW2L);

        osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
        osg::ref_ptr<osg::Vec3Array> uvs = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
        osg::ref_ptr<osg::DrawElementsUInt> de = new osg::DrawElementsUInt(GL_TRIANGLES);

        auto parentVertToWorld = [&](unsigned vi) -> osg::Vec3d {
            const osg::Vec3& pl = (*parentMesh.verts)[vi];
            return osg::Vec3d(pl.x(), pl.y(), pl.z()) * parentMesh.localToWorld;
        };

        for (unsigned t = 0; t < parentMesh.indices->getNumIndices(); t += 3u)
        {
            const unsigned i0 = parentMesh.indices->getElement(t);
            const unsigned i1 = parentMesh.indices->getElement(t + 1u);
            const unsigned i2 = parentMesh.indices->getElement(t + 2u);

            const osg::Vec3d w0 = parentVertToWorld(i0);
            const osg::Vec3d w1 = parentVertToWorld(i1);
            const osg::Vec3d w2 = parentVertToWorld(i2);
            const osg::Vec3d wc = (w0 + w1 + w2) * (1.0 / 3.0);

            osg::Vec3d lla;
            if (!horiz->transformFromWorld(wc, lla))
                continue;
            if (!childEx.contains(lla.x(), lla.y(), horiz))
                continue;

            const auto emitVert = [&](unsigned pi) -> void {
                const osg::Vec3d world = parentVertToWorld(pi);
                const osg::Vec3d cl = world * childW2L;
                osg::Vec3d unit;
                childLocator.worldToUnit(world, unit);
                verts->push_back(osg::Vec3((float)cl.x(), (float)cl.y(), (float)cl.z()));
                const int marker = (int)(*parentMesh.uvs)[pi].z();
                uvs->push_back(osg::Vec3((float)unit.x(), (float)unit.y(), (float)marker));
            };

            const unsigned base = verts->size();
            emitVert(i0);
            emitVert(i1);
            emitVert(i2);
            de->addElement(base);
            de->addElement(base + 1u);
            de->addElement(base + 2u);
        }

        if (verts->empty() || de->getNumIndices() < 3u)
            return out;

        osg::ref_ptr<osg::VertexBufferObject> vbo = new osg::VertexBufferObject();
        verts->setVertexBufferObject(vbo.get());
        uvs->setVertexBufferObject(vbo.get());

        osg::ref_ptr<osg::Vec3Array> normals;
        {
            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
            geometry->setVertexArray(verts.get());
            geometry->addPrimitiveSet(de.get());
            geode->addDrawable(geometry.get());
            osgUtil::SmoothingVisitor sv;
            geode->accept(sv);
            normals = static_cast<osg::Vec3Array*>(geometry->getNormalArray());
            if (normals.valid())
                normals->setVertexBufferObject(vbo.get());
        }

        out.verts = verts.get();
        out.uvs = uvs.get();
        out.normals = normals.get();
        out.indices = de.get();
        out.localToWorld = childL2W;
        return out;
    }
}

void CesiumIonTerrainMeshLayer::setLogTerrainHttpResponses(bool value)
{
    s_logTerrainHttpResponses.store(value, std::memory_order_relaxed);
}

bool CesiumIonTerrainMeshLayer::getLogTerrainHttpResponses()
{
    return s_logTerrainHttpResponses.load(std::memory_order_relaxed);
}

TileMesh CesiumIonTerrainMeshLayer::createTileImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
    // ── 서킷 브레이커 (fastFail 모드에서만 활성) ────────────────────────────
    // 서버 오류 후 30초간 요청을 완전히 건너뛴다. HTTP 스택에 도달하지 않으므로
    // 죽은 서버에 반복 요청을 보내지 않고 즉시 폴백 레이어로 넘어간다.
    if (_fastFailOnServerError && _circuitOpen.load(std::memory_order_relaxed))
    {
        const int64_t now = std::chrono::steady_clock::now().time_since_epoch().count();
        if (now < _circuitRetryNs.load(std::memory_order_relaxed))
            return TileMesh(); // 쿨다운 중 — 요청 없이 즉시 반환
        // 쿨다운 만료 → 회로 닫고 한 번 프로브
        _circuitOpen.store(false, std::memory_order_relaxed);
    }

    ReadResult::Code code = ReadResult::RESULT_NOT_FOUND;
    TileMesh mesh = tryReadCesiumQuantizedMeshForKey(key, _assetURI, getReadOptions(), progress, &code, _fastFailOnServerError);
    if (mesh.verts.valid())
    {
        applyConstraints(key, mesh);
        return mesh;
    }

    // fastFailOnServerError: 연결 오류 → 서킷 열고 즉시 반환
    if (_fastFailOnServerError && code != ReadResult::RESULT_NOT_FOUND)
    {
        _circuitOpen.store(true, std::memory_order_relaxed);
        const int64_t retryAt = (std::chrono::steady_clock::now().time_since_epoch()
                                 + std::chrono::nanoseconds(_kCircuitCooldownNs)).count();
        _circuitRetryNs.store(retryAt, std::memory_order_relaxed);
        return TileMesh();
    }

    constexpr unsigned kMaxParentWalk = 16u;
    TileKey parentKey = key.createParentKey();
    for (unsigned hop = 0; hop < kMaxParentWalk && parentKey.valid(); ++hop, parentKey = parentKey.createParentKey())
    {
        if (progress && progress->isCanceled())
            return TileMesh();

        TileMesh parentMesh = tryReadCesiumQuantizedMeshForKey(parentKey, _assetURI, getReadOptions(), progress, &code, _fastFailOnServerError);
        if (!parentMesh.verts.valid())
        {
            // fastFailOnServerError: 서버 오류면 부모 워크도 즉시 중단
            if (_fastFailOnServerError && code != ReadResult::RESULT_NOT_FOUND)
                return TileMesh();
            continue;
        }

        TileMesh upsampled = clipParentTerrainMeshToChildTile(key, parentMesh);
        if (upsampled.verts.valid() && upsampled.verts->size() >= 3u)
        {
            applyConstraints(key, upsampled);
            return upsampled;
        }
    }

    return TileMesh();
}
