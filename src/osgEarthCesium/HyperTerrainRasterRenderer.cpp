#include "HyperTerrainRasterRenderer"
// WINGDIAPI/APIENTRY and std::result_of compat provided by ForcedInclude compat_result_of.h

#include <CesiumGltf/ImageAsset.h>
#include <CesiumRasterOverlays/RasterOverlayTile.h>

#include <osg/GL>
#include <osg/Notify>
#include <osg/Texture2D>
#include <osg/Image>

#include <cstring>
#include <vector>

namespace osgEarth { namespace Cesium {

namespace {

// 워커 스레드에서 복사한 이미지 데이터 (메인 스레드까지 안전하게 전달)
struct RasterLoadData {
    int32_t width    = 0;
    int32_t height   = 0;
    int32_t channels = 0;
    std::vector<std::byte> pixelData;
};

} // namespace

void* HyperTerrainRasterRenderer::prepareRasterInLoadThread(
    CesiumGltf::ImageAsset& image,
    const std::any& /*rendererOptions*/)
{
    if (image.pixelData.empty()) {
        OSG_WARN << "[HyperTerrain] prepareRasterInLoadThread: empty pixelData" << std::endl;
        return nullptr;
    }

    // 워커 스레드에서 이미지를 복사 → rasterTile.getImage()에 의존하지 않음
    auto* data = new RasterLoadData();
    data->width     = image.width;
    data->height    = image.height;
    data->channels  = image.channels;
    data->pixelData = image.pixelData; // copy
    return data;
}

void* HyperTerrainRasterRenderer::prepareRasterInMainThread(
    CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pLoadThreadResult)
{
    // 워커 스레드에서 복사한 데이터 우선 사용; 없으면 rasterTile.getImage() 폴백
    int width = 0, height = 0, channels = 0;
    const std::byte* pixelPtr  = nullptr;
    size_t           pixelSize = 0;

    RasterLoadData* pData = static_cast<RasterLoadData*>(pLoadThreadResult);
    if (pData && !pData->pixelData.empty()) {
        width     = pData->width;
        height    = pData->height;
        channels  = pData->channels;
        pixelPtr  = pData->pixelData.data();
        pixelSize = pData->pixelData.size();
    } else {
        // 폴백: rasterTile.getImage()
        const CesiumGltf::ImageAsset* pImage = rasterTile.getImage();
        if (!pImage || pImage->pixelData.empty()) {
            OSG_WARN << "[HyperTerrain] prepareRasterInMainThread: no image data "
                     << "(loadResult=" << (pLoadThreadResult ? "non-null-but-empty" : "null")
                     << " getImage=" << (pImage ? "non-null-but-empty" : "null") << ")" << std::endl;
            delete pData;
            return nullptr;
        }
        width     = pImage->width;
        height    = pImage->height;
        channels  = pImage->channels;
        pixelPtr  = pImage->pixelData.data();
        pixelSize = pImage->pixelData.size();
    }

    if (width <= 0 || height <= 0) {
        OSG_WARN << "[HyperTerrain] prepareRasterInMainThread: invalid dimensions "
                 << width << "x" << height << std::endl;
        delete pData;
        return nullptr;
    }

    const size_t nPix = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (nPix == 0 || pixelSize < nPix) {
        OSG_WARN << "[HyperTerrain] prepareRasterInMainThread: pixel buffer too small" << std::endl;
        delete pData;
        return nullptr;
    }

    // image.channels 가 실제 버퍼와 불일치하면(예: RGBA PNG 인데 channels==3)
    // GL_RGB 로 올라가 셰이더에서 .a 가 항상 1 이 되어 오버레이가 불투명해진다.
    const size_t bpp = pixelSize / nPix;
    if (bpp * nPix != pixelSize) {
        OSG_WARN << "[HyperTerrain] prepareRasterInMainThread: pixelSize not divisible by w*h" << std::endl;
        delete pData;
        return nullptr;
    }

    std::vector<std::byte> expandedLaToRgba;
    const std::byte* uploadPtr = pixelPtr;
    size_t           uploadLen = pixelSize;

    if (bpp == 4 && channels == 3) {
        channels = 4;
    }

    if (bpp == 2) {
        expandedLaToRgba.resize(nPix * 4u);
        for (size_t i = 0; i < nPix; ++i) {
            const std::byte L = pixelPtr[i * 2u];
            const std::byte A = pixelPtr[i * 2u + 1u];
            expandedLaToRgba[i * 4u + 0u] = L;
            expandedLaToRgba[i * 4u + 1u] = L;
            expandedLaToRgba[i * 4u + 2u] = L;
            expandedLaToRgba[i * 4u + 3u] = A;
        }
        uploadPtr = expandedLaToRgba.data();
        uploadLen = expandedLaToRgba.size();
        channels  = 4;
    }

    GLenum pixelFmt = GL_RGBA;
    GLenum dataType = GL_UNSIGNED_BYTE;
    size_t outBytes = 0;

    if (uploadLen >= nPix * 4u && (channels == 4 || bpp == 4 || bpp == 2)) {
        pixelFmt = GL_RGBA;
        outBytes = nPix * 4u;
    } else if (uploadLen >= nPix * 3u && channels == 3 && bpp == 3) {
        pixelFmt = GL_RGB;
        outBytes = nPix * 3u;
    } else if (uploadLen >= nPix && channels == 1 && bpp == 1) {
        pixelFmt = GL_RED;
        outBytes = nPix;
    } else {
        OSG_WARN << "[HyperTerrain] prepareRasterInMainThread: unsupported raster layout "
                 << "channels=" << channels << " bpp=" << bpp << std::endl;
        delete pData;
        return nullptr;
    }

    if (uploadLen < outBytes) {
        OSG_WARN << "[HyperTerrain] prepareRasterInMainThread: buffer shorter than expected "
                 << uploadLen << " < " << outBytes << std::endl;
        delete pData;
        return nullptr;
    }

    auto* osgImage = new osg::Image();
    osgImage->allocateImage(width, height, 1, pixelFmt, dataType);
    std::memcpy(osgImage->data(), uploadPtr, outBytes);
    osgImage->flipVertical(); // Cesium 픽셀 좌표 → OSG 픽셀 좌표

    auto* tex = new osg::Texture2D(osgImage);
    tex->setResizeNonPowerOfTwoHint(false);
    tex->setInternalFormatMode(osg::Texture::USE_USER_DEFINED_FORMAT);
    if (pixelFmt == GL_RGBA) {
        tex->setInternalFormat(GL_RGBA8);
    } else if (pixelFmt == GL_RGB) {
        tex->setInternalFormat(GL_RGB8);
    } else {
        tex->setInternalFormat(GL_R8);
    }
    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    tex->ref(); // 소유권: freeRaster에서 unref()

    delete pData; // pLoadThreadResult 해제
    return tex;
}

void HyperTerrainRasterRenderer::freeRaster(
    const CesiumRasterOverlays::RasterOverlayTile& /*rasterTile*/,
    void* pLoadThreadResult,
    void* pMainThreadResult) noexcept
{
    // state==Done 시: pLoadThreadResult=null, pMainThreadResult=tex → tex 해제
    // state<Done 시: pLoadThreadResult=RasterLoadData*, pMainThreadResult=null → data 해제
    if (pLoadThreadResult) {
        delete static_cast<RasterLoadData*>(pLoadThreadResult);
    }
    if (pMainThreadResult) {
        auto* tex = static_cast<osg::Texture2D*>(pMainThreadResult);
        tex->unref_nodelete();
    }
}

} // namespace Cesium
} // namespace osgEarth
