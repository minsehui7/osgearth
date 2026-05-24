// WINGDIAPI/APIENTRY and std::result_of compat provided by ForcedInclude compat_result_of.h

#include "HyperTerrainPrepareRendererResources"
#include "HyperTerrainImageryFactory"
#include "HyperTerrainRasterRenderer"

#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/TileContent.h>
#include <Cesium3DTilesSelection/TileLoadResult.h>
#include <Cesium3DTilesSelection/RasterMappedTo3DTile.h>
#include <CesiumAsync/AsyncSystem.h>
#include <CesiumGltf/Model.h>
#include <CesiumGltf/Accessor.h>
#include <CesiumGltf/MeshPrimitive.h>
#include <CesiumGltf/AccessorView.h>
#include <CesiumRasterOverlays/RasterOverlayTile.h>

#include <algorithm>
#include <cmath>

#include <osg/BlendFunc>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/Texture2D>
#include <osg/Notify>
#include <osg/Program>
#include <osg/Shader>
#include <osg/RenderInfo>

#include <osgEarth/Lighting>

#include <glm/mat4x4.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/geometric.hpp>

namespace osgEarth { namespace Cesium {

namespace {

// prepareInLoadThread → prepareInMainThread 데이터
struct LoadThreadData {
    glm::dmat4 transform; // tile local → ECEF 변환 (prepareInLoadThread에서 쫭처)
};

// 최대 동시 오버레이 슬롯 수
static constexpr int kMaxOverlays = 4;

// osgEarth SimpleSky.Ground.ONeil.* (sky_simple) 과 동일 계열. 타일은 leaf Program 이라 VP 합성 없음.
const char* kSimpleVertGLSL = R"GLSL(
#version 330 core

#define OE_NUM_LIGHTS 1

uniform mat4 osg_ModelViewProjectionMatrix;
uniform mat4 osg_ModelViewMatrix;
uniform mat3 osg_NormalMatrix;
uniform mat4 osg_ViewMatrix;
uniform mat4 osg_ViewMatrixInverse;

uniform float atmos_fOuterRadius;
uniform float atmos_fInnerRadius;

// 거리 헤이즈 uniforms (MapNode StateSet에서 상속).
// hl_fog_end <= 0  → 비활성. ViewOsgEarth.cpp::installDistanceFog() 참고.
uniform vec3  hl_fog_center;
uniform float hl_fog_start;
uniform float hl_fog_end;

struct osg_LightSourceParameters {
   vec4 ambient;
   vec4 diffuse;
   vec4 specular;
   vec4 position;
   vec3 spotDirection;
   float spotExponent;
   float spotCutoff;
   float spotCosCutoff;
   float constantAttenuation;
   float linearAttenuation;
   float quadraticAttenuation;
   bool enabled;
};
uniform osg_LightSourceParameters osg_LightSource[OE_NUM_LIGHTS];

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_texcoord0;
layout(location = 3) in vec2 a_texcoord1;
layout(location = 4) in vec2 a_texcoord2;
layout(location = 5) in vec2 a_texcoord3;

const float ATM_PI = 3.1415927;
const float ATM_Kr = 0.0025;
const float ATM_Km = 0.0015;
const float ATM_ESun = 15.0;
const float ATM_RSDepth = 0.25;
const float ATM_fKrESun = ATM_Kr * ATM_ESun;
const float ATM_fKmESun = ATM_Km * ATM_ESun;
const float ATM_fKr4PI = ATM_Kr * 4.0 * ATM_PI;
const float ATM_fKm4PI = ATM_Km * 4.0 * ATM_PI;
const vec3  ATM_v3InvWavelength = vec3(5.6020447, 9.4732844, 19.6438026);
#define ATM_N_SAMPLES 2
#define ATM_F_SAMPLES 2.0

out vec3 v_normalEye;
out vec2 v_tc[4];
out vec3 vp_VertexView;
out vec3 atmos_lightDir;
out vec3 atmos_color;
out vec3 atmos_up;
out float atmos_space;
out float v_hazeFactor;
out vec4 v_oeOverlayTexcoord;

uniform mat4 oe_overlay_texmatrix;

float atmos_scale_scatter(float fCos) {
    float x = 1.0 - fCos;
    return ATM_RSDepth * exp(-0.00287 + x*(0.459 + x*(3.83 + x*(-6.80 + x*5.25))));
}

void atmos_GroundFromSpace(vec4 vertexVIEW,
    vec3 earthCenter,
    vec3 normal,
    float camH2)
{
    vec3 v3Pos = vertexVIEW.xyz;
    vec3 v3Ray = v3Pos;
    float fFar = length(v3Ray);
    v3Ray /= fFar;

    float B = 2.0 * dot(-earthCenter, v3Ray);
    float C = camH2 - atmos_fOuterRadius * atmos_fOuterRadius;
    float fDet = max(0.0, B*B - 4.0*C);
    float fNear = 0.5 * (-B - sqrt(fDet));

    vec3 v3Start = v3Ray * fNear;
    fFar -= fNear;
    float fDepth = exp((atmos_fInnerRadius - atmos_fOuterRadius) / ATM_RSDepth);
    float fCameraAngle = dot(-v3Ray, normal);
    float fLightAngle = dot(atmos_lightDir, normal);
    float fCameraScale = atmos_scale_scatter(fCameraAngle);
    float fLightScale = atmos_scale_scatter(fLightAngle);
    float fCameraOffset = fDepth * fCameraScale;
    float fTemp = fLightScale * fCameraScale;

    float atmos_fScale = 1.0 / (atmos_fOuterRadius - atmos_fInnerRadius);
    float atmos_fScaleOverScaleDepth = atmos_fScale / ATM_RSDepth;

    float fSampleLength = fFar / ATM_F_SAMPLES;
    float fScaledLength = fSampleLength * atmos_fScale;
    vec3 v3SampleRay = v3Ray * fSampleLength;
    vec3 v3SamplePoint = v3Start + v3SampleRay * 0.5;

    vec3 v3FrontColor = vec3(0.0);
    for (int i = 0; i < ATM_N_SAMPLES; ++i) {
        float fHeight = length(v3SamplePoint - earthCenter);
        float fDepthI = exp(atmos_fScaleOverScaleDepth * (atmos_fInnerRadius - fHeight));
        float fScatter = fDepthI * fTemp - fCameraOffset;
        vec3 v3Attenuate = exp(-fScatter * (ATM_v3InvWavelength * ATM_fKr4PI + ATM_fKm4PI));
        v3FrontColor += v3Attenuate * (fDepthI * fScaledLength);
        v3SamplePoint += v3SampleRay;
    }
    atmos_color = v3FrontColor * (ATM_v3InvWavelength * ATM_fKrESun + ATM_fKmESun);
}

void atmos_GroundFromAtmosphere(vec4 vertexVIEW, vec3 earthCenter, vec3 normal, float camH2)
{
    vec3 v3Pos = vertexVIEW.xyz / vertexVIEW.w;
    vec3 v3Ray = v3Pos;
    float fFar = length(v3Ray);
    v3Ray /= fFar;

    float atmos_fScale = 1.0 / (atmos_fOuterRadius - atmos_fInnerRadius);
    float atmos_fScaleOverScaleDepth = atmos_fScale / ATM_RSDepth;
    float camH = sqrt(max(camH2, 0.0));

    float fDepth = exp((atmos_fInnerRadius - camH) / ATM_RSDepth);
    float fCameraAngle = max(0.0, dot(-v3Ray, normal));
    float fLightAngle = dot(atmos_lightDir, normal);
    float fCameraScale = atmos_scale_scatter(fCameraAngle);
    float fLightScale = atmos_scale_scatter(fLightAngle);
    float fCameraOffset = fDepth * fCameraScale;
    float fTemp = fLightScale * fCameraScale;

    float fSampleLength = fFar / ATM_F_SAMPLES;
    float fScaledLength = fSampleLength * atmos_fScale;
    vec3 v3SampleRay = v3Ray * fSampleLength;
    vec3 v3SamplePoint = v3SampleRay * 0.5;

    vec3 v3FrontColor = vec3(0.0);
    for (int i = 0; i < ATM_N_SAMPLES; ++i) {
        float fHeight = length(v3SamplePoint - earthCenter);
        float fDepthI = exp(atmos_fScaleOverScaleDepth * (atmos_fInnerRadius - fHeight));
        float fScatter = fDepthI * fTemp - fCameraOffset;
        vec3 v3Attenuate = exp(-fScatter * (ATM_v3InvWavelength * ATM_fKr4PI + ATM_fKm4PI));
        v3FrontColor += v3Attenuate * (fDepthI * fScaledLength);
        v3SamplePoint += v3SampleRay;
    }
    atmos_color = v3FrontColor * (ATM_v3InvWavelength * ATM_fKrESun + ATM_fKmESun);
}

void main() {
    vec4 vertexVIEW = osg_ModelViewMatrix * vec4(a_position, 1.0);
    gl_Position = osg_ModelViewProjectionMatrix * vec4(a_position, 1.0);
    v_oeOverlayTexcoord = oe_overlay_texmatrix * vertexVIEW;
    v_normalEye = normalize(osg_NormalMatrix * a_normal);
    v_tc[0] = a_texcoord0;
    v_tc[1] = a_texcoord1;
    v_tc[2] = a_texcoord2;
    v_tc[3] = a_texcoord3;

    vp_VertexView = vertexVIEW.xyz / max(vertexVIEW.w, 1e-6);

    // 거리 헤이즈 인자: ECEF world-space 좌표 기준으로 hl_fog_center 까지의 거리.
    // hl_fog_end <= 0 이면 비활성 (ViewOsgEarth.cpp::installDistanceFog 초기값).
    if (hl_fog_end > 0.0) {
        vec4 worldPos4 = osg_ViewMatrixInverse * vertexVIEW;
        vec3 worldPos  = worldPos4.xyz / max(worldPos4.w, 1e-6);
        float dist = length(worldPos - hl_fog_center);
        float range = max(hl_fog_end - hl_fog_start, 1.0);
        v_hazeFactor = clamp((dist - hl_fog_start) / range, 0.0, 1.0);
    } else {
        v_hazeFactor = 0.0;
    }

    if (!osg_LightSource[0].enabled) {
        atmos_lightDir = normalize(vec3(0.45, 0.62, 0.64));
        atmos_color = vec3(0.0);
        atmos_up = vec3(0.0, 0.0, 1.0);
        atmos_space = 0.0;
        return;
    }

    atmos_lightDir = normalize(osg_LightSource[0].position.xyz);

    vec4 ec4 = osg_ViewMatrix * vec4(0.0, 0.0, 0.0, 1.0);
    vec3 earthCenter = ec4.xyz / max(ec4.w, 1e-6);
    vec3 v3Surf = vp_VertexView - earthCenter;
    float surfLen = length(v3Surf);
    vec3 normal = surfLen > 1e-5 ? v3Surf / surfLen : vec3(0.0, 0.0, 1.0);
    atmos_up = normal;

    float camH = length(osg_ViewMatrixInverse[3].xyz);
    float camH2 = camH * camH;
    atmos_space = max(0.0, (camH - atmos_fInnerRadius)
        / max(atmos_fOuterRadius - atmos_fInnerRadius, 1e-6));

    if (camH >= atmos_fOuterRadius) {
        atmos_GroundFromSpace(vertexVIEW, earthCenter, normal, camH2);
    } else {
        atmos_GroundFromAtmosphere(vertexVIEW, earthCenter, normal, camH2);
    }
}
)GLSL";

const char* kSimpleFragGLSL = R"GLSL(
#version 330 core

#define OE_NUM_LIGHTS 1

uniform sampler2D u_overlayTex[4];
uniform vec2 u_overlayTrans[4];
uniform vec2 u_overlayScale[4];
uniform float u_overlayAlpha[4];
uniform int  u_overlayActive[4];

uniform float u_tmsImageryExposure;
uniform float oe_sky_ambientBoostFactor;

// 거리 헤이즈 fragment uniform (MapNode StateSet에서 상속).
// hl_fog_visual_scale: SkyNode 활성 시 헤이즈 강도를 줄이는 비주얼 스케일.
// hl_fog_haze_color: ViewOsgEarth::updateDistanceFogUniforms 가 MapNode StateSet 에서 갱신(주·야 블렌드).
uniform float hl_fog_visual_scale;
uniform vec3  hl_fog_haze_color;

struct osg_LightSourceParameters {
   vec4 ambient;
   vec4 diffuse;
   vec4 specular;
   vec4 position;
   vec3 spotDirection;
   float spotExponent;
   float spotCutoff;
   float spotCosCutoff;
   float constantAttenuation;
   float linearAttenuation;
   float quadraticAttenuation;
   bool enabled;
};
uniform osg_LightSourceParameters osg_LightSource[OE_NUM_LIGHTS];

struct osg_MaterialParameters {
   vec4 emission;
   vec4 ambient;
   vec4 diffuse;
   vec4 specular;
   float shininess;
};
uniform osg_MaterialParameters osg_FrontMaterial;

in vec3 v_normalEye;
in vec2 v_tc[4];
in vec3 vp_VertexView;
in vec3 atmos_lightDir;
in vec3 atmos_color;
in vec3 atmos_up;
in float atmos_space;
in float v_hazeFactor;
in vec4 v_oeOverlayTexcoord;

uniform sampler2D oe_overlay_tex;
uniform float oe_overlay_ready;

out vec4 fragColor;

const vec3 kBaseColor = vec3(0.60, 0.50, 0.40);

// 거리 헤이즈 적용 — 원본 wip/osgearth-integration::hl_fog_frag 와 동일한 톤매핑.
// rgb 가 display-space (post tone-map) 일 때 호출.
vec3 applyDistanceHaze(vec3 rgb) {
    if (v_hazeFactor <= 0.0) return rgb;
    float f = clamp(v_hazeFactor * hl_fog_visual_scale, 0.0, 1.0);
    if (f <= 0.0) return rgb;
    float lum = dot(rgb, vec3(0.299, 0.587, 0.114));
    vec3 desat = mix(rgb, vec3(lum), f * 0.6);
    return mix(desat, hl_fog_haze_color, f);
}

// 카메라가 지표에 가까울수록(atmos_space 작음) 주·야 경계(확산 감쇠)를 완만하게 넓힘 — 피크 강도는 유지.
float dayTermHorizonWidth() {
    const float spreadMax = 0.62;
    const float spaceFull = 0.70;
    return spreadMax * (1.0 - smoothstep(0.0, spaceFull, atmos_space));
}

void applyDrapeOverlay(inout vec3 colorRgb) {
    // osgEarth DrapingTechnique: oe_overlay_tex is projective over the globe mesh.
    // Keep it between base imagery and transparent weather/label overlays.
    if (oe_overlay_ready > 0.5 && v_oeOverlayTexcoord.w > 1e-6) {
        vec2 pq = v_oeOverlayTexcoord.xy / v_oeOverlayTexcoord.w;
        if (pq.x >= 0.0 && pq.x <= 1.0 && pq.y >= 0.0 && pq.y <= 1.0) {
            vec4 drapeColor = textureProj(oe_overlay_tex, v_oeOverlayTexcoord);
            float drapeLuma = max(max(drapeColor.r, drapeColor.g), drapeColor.b);
            if (drapeColor.a > 1.0 / 255.0 && drapeLuma > 1.0 / 255.0) {
                colorRgb = mix(colorRgb, drapeColor.rgb, drapeColor.a);
            }
        }
    }
}

vec3 applyTmsImageryExposure(vec3 rgb) {
    return vec3(1.0) - exp(-u_tmsImageryExposure * 0.33 * rgb);
}

void main() {
    vec3 colorRgb = kBaseColor;
    // sampler2D 배열을 비상수 인덱스로 texture() 하면 일부 드라이버에서 비정상 샘플링된다.
    // 슬롯마다 상수 인덱스로 펼친다 (OWM 구름 등 RGBA 알파 유지).
    if (u_overlayActive[0] != 0) {
        vec2 uv0 = v_tc[0] * u_overlayScale[0] + u_overlayTrans[0];
        vec4 tc0 = texture(u_overlayTex[0], uv0);
        tc0.rgb = applyTmsImageryExposure(tc0.rgb);
        tc0.a *= u_overlayAlpha[0];
        colorRgb = mix(colorRgb, tc0.rgb, tc0.a);
    }

    applyDrapeOverlay(colorRgb);

    if (u_overlayActive[1] != 0) {
        vec2 uv1 = v_tc[1] * u_overlayScale[1] + u_overlayTrans[1];
        vec4 tc1 = texture(u_overlayTex[1], uv1);
        tc1.rgb = applyTmsImageryExposure(tc1.rgb);
        tc1.a *= u_overlayAlpha[1];
        colorRgb = mix(colorRgb, tc1.rgb, tc1.a);
    }
    if (u_overlayActive[2] != 0) {
        vec2 uv2 = v_tc[2] * u_overlayScale[2] + u_overlayTrans[2];
        vec4 tc2 = texture(u_overlayTex[2], uv2);
        tc2.rgb = applyTmsImageryExposure(tc2.rgb);
        tc2.a *= u_overlayAlpha[2];
        colorRgb = mix(colorRgb, tc2.rgb, tc2.a);
    }
    if (u_overlayActive[3] != 0) {
        vec2 uv3 = v_tc[3] * u_overlayScale[3] + u_overlayTrans[3];
        vec4 tc3 = texture(u_overlayTex[3], uv3);
        tc3.rgb = applyTmsImageryExposure(tc3.rgb);
        tc3.a *= u_overlayAlpha[3];
        colorRgb = mix(colorRgb, tc3.rgb, tc3.a);
    }

    vec3 N = normalize(v_normalEye);

    if (!osg_LightSource[0].enabled) {
        vec3 L = normalize(vec3(0.45, 0.62, 0.64));
        float diff = max(dot(N, L), 0.0);
        float light = 0.25 + 0.75 * diff;
        fragColor = vec4(applyDistanceHaze(colorRgb * light), 1.0);
        return;
    }

    vec4 color = vec4(colorRgb, 1.0);

    vec3 U = normalize(atmos_up);

    vec3 totalDiffuse = vec3(0.0);
    vec3 totalAmbient = vec3(0.0);
    vec3 totalSpecular = vec3(0.0);

    float shine = clamp(osg_FrontMaterial.shininess, 1.0, 128.0);
    vec3 surfaceSpecularity = osg_FrontMaterial.specular.rgb;

    float horizW = dayTermHorizonWidth();

    for (int li = 0; li < OE_NUM_LIGHTS; ++li) {
        if (osg_LightSource[li].enabled)
        {
            float attenuation = 1.0;
            vec3 L = normalize(osg_LightSource[li].position.xyz);
            vec3 V = -normalize(vp_VertexView);

            if (osg_LightSource[li].position.w != 0.0) {
                vec3 Lu = osg_LightSource[li].position.xyz - vp_VertexView;
                float distance = length(Lu);
                attenuation = 1.0 / (
                    osg_LightSource[li].constantAttenuation +
                    osg_LightSource[li].linearAttenuation * distance +
                    osg_LightSource[li].quadraticAttenuation * distance * distance);
                L = normalize(Lu);

                if (osg_LightSource[li].spotCutoff <= 90.0) {
                    vec3 D = normalize(osg_LightSource[li].spotDirection);
                    float clampedCos = max(0.0, dot(-L, D));
                    attenuation = clampedCos < osg_LightSource[li].spotCosCutoff
                        ? 0.0
                        : attenuation * pow(clampedCos, osg_LightSource[li].spotExponent);
                }
            }

            float uDotL = dot(U, L);
            // 낮/밝기 보조: 원래 cos(zenith) 그대로 — 범위만 넓히지 않음
            float dayTermPeak = uDotL;
            float dayTermWide = smoothstep(-horizW, 1.0, uDotL);

            float dayTerm = li == 0 ? dayTermPeak : 1.0;
            float ambientBoost = li == 0
                ? 1.0 + oe_sky_ambientBoostFactor * clamp(2.0 * (dayTerm - 0.5), 0.0, 1.0)
                : 1.0;

            vec3 ambientReflection = attenuation
                * osg_LightSource[li].ambient.rgb
                * ambientBoost;

            float NdotL = max(dot(N, L), 0.0);
            float diffuseAttenuation = li == 0
                ? clamp(dayTermWide + 0.35, 0.0, 1.0)
                : 1.0;

            vec3 diffuseReflection =
                attenuation * diffuseAttenuation * osg_LightSource[li].diffuse.rgb * NdotL;

            vec3 specularReflection = vec3(0.0);
            if (NdotL > 0.0) {
                float specAttenuation = clamp(NdotL * 10.0, 0.0, 1.0);
                vec3 H = reflect(-L, N);
                float HdotV = max(dot(H, V), 0.0);
                specularReflection = specAttenuation * attenuation
                    * osg_LightSource[li].specular.rgb * surfaceSpecularity
                    * pow(HdotV, shine);
            }

            totalDiffuse += diffuseReflection;
            totalAmbient += ambientReflection;
            totalSpecular += specularReflection;
        }
    }

    color.rgb += atmos_color;

    vec3 lightColor =
        osg_FrontMaterial.emission.rgb +
        totalDiffuse * osg_FrontMaterial.diffuse.rgb +
        totalAmbient * osg_FrontMaterial.ambient.rgb;

    color.rgb = color.rgb * lightColor + totalSpecular;

    // 거리 헤이즈는 톤매핑 후 (display-space) 에 적용 — 원본 osgEarth VirtualProgram
    // hl_fog_frag(LOCATION_FRAGMENT_COLORING) 와 같은 단계.
    color.rgb = applyDistanceHaze(color.rgb);

    fragColor = vec4(color.rgb, 1.0);
}
)GLSL";

// glTF NORMAL accessor 추출 또는 삼각형에서 면적 가중 법선 계산
osg::Vec3Array* buildNormals(
    const CesiumGltf::Model& model,
    const CesiumGltf::MeshPrimitive& prim,
    const CesiumGltf::AccessorView<glm::vec3>& positions)
{
    const size_t nVerts = static_cast<size_t>(positions.size());

    // 1) glTF NORMAL accessor 시도
    auto normIt = prim.attributes.find("NORMAL");
    if (normIt != prim.attributes.end() && normIt->second >= 0
        && static_cast<size_t>(normIt->second) < model.accessors.size()) {
        CesiumGltf::AccessorView<glm::vec3> normView(
            model, model.accessors[static_cast<size_t>(normIt->second)]);
        if (normView.status() == CesiumGltf::AccessorViewStatus::Valid
            && static_cast<size_t>(normView.size()) == nVerts) {
            auto* arr = new osg::Vec3Array(nVerts);
            for (size_t i = 0; i < nVerts; ++i)
                (*arr)[i].set(normView[static_cast<int64_t>(i)].x,
                              normView[static_cast<int64_t>(i)].y,
                              normView[static_cast<int64_t>(i)].z);
            return arr;
        }
    }

    // 2) 삼각형에서 면적 가중 법선 계산
    std::vector<glm::vec3> normals(nVerts, glm::vec3(0.0f));
    auto accumTri = [&](size_t i0, size_t i1, size_t i2) {
        const glm::vec3 v0(positions[static_cast<int64_t>(i0)].x,
                           positions[static_cast<int64_t>(i0)].y,
                           positions[static_cast<int64_t>(i0)].z);
        const glm::vec3 v1(positions[static_cast<int64_t>(i1)].x,
                           positions[static_cast<int64_t>(i1)].y,
                           positions[static_cast<int64_t>(i1)].z);
        const glm::vec3 v2(positions[static_cast<int64_t>(i2)].x,
                           positions[static_cast<int64_t>(i2)].y,
                           positions[static_cast<int64_t>(i2)].z);
        const glm::vec3 n = glm::cross(v1 - v0, v2 - v0);
        normals[i0] += n;
        normals[i1] += n;
        normals[i2] += n;
    };

    if (prim.indices >= 0
        && static_cast<size_t>(prim.indices) < model.accessors.size()) {
        const auto& idxAcc = model.accessors[static_cast<size_t>(prim.indices)];
        CesiumGltf::AccessorView<uint16_t> idx16(model, idxAcc);
        if (idx16.status() == CesiumGltf::AccessorViewStatus::Valid) {
            for (int64_t i = 0; i + 2 < idx16.size(); i += 3)
                accumTri(idx16[i], idx16[i+1], idx16[i+2]);
        } else {
            CesiumGltf::AccessorView<uint32_t> idx32(model, idxAcc);
            if (idx32.status() == CesiumGltf::AccessorViewStatus::Valid)
                for (int64_t i = 0; i + 2 < idx32.size(); i += 3)
                    accumTri(idx32[i], idx32[i+1], idx32[i+2]);
        }
    } else {
        for (size_t i = 0; i + 2 < nVerts; i += 3)
            accumTri(i, i+1, i+2);
    }

    auto* arr = new osg::Vec3Array(nVerts);
    for (size_t i = 0; i < nVerts; ++i) {
        const float len = glm::length(normals[i]);
        (*arr)[i] = (len > 1e-6f)
            ? osg::Vec3f(normals[i].x / len, normals[i].y / len, normals[i].z / len)
            : osg::Vec3f(0.0f, 1.0f, 0.0f); // Y-up default
    }
    return arr;
}

osg::ref_ptr<osg::Program> buildSimpleShader() {
    auto* prog = new osg::Program();
    prog->setName("HyperTerrainLitAtmosphere");
    prog->addShader(new osg::Shader(osg::Shader::VERTEX,   kSimpleVertGLSL));
    prog->addShader(new osg::Shader(osg::Shader::FRAGMENT, kSimpleFragGLSL));
    prog->addBindAttribLocation("a_position",  0);
    prog->addBindAttribLocation("a_normal",    1);
    prog->addBindAttribLocation("a_texcoord0", 2);
    prog->addBindAttribLocation("a_texcoord1", 3);
    prog->addBindAttribLocation("a_texcoord2", 4);
    prog->addBindAttribLocation("a_texcoord3", 5);
    return prog;
}

// 오버레이 유니폼을 기본값(비활성)으로 초기화
void initOverlayUniforms(osg::StateSet* ss) {
    auto* texU    = new osg::Uniform(osg::Uniform::SAMPLER_2D, "u_overlayTex",    kMaxOverlays);
    auto* transU  = new osg::Uniform(osg::Uniform::FLOAT_VEC2, "u_overlayTrans",  kMaxOverlays);
    auto* scaleU  = new osg::Uniform(osg::Uniform::FLOAT_VEC2, "u_overlayScale",  kMaxOverlays);
    auto* alphaU  = new osg::Uniform(osg::Uniform::FLOAT,      "u_overlayAlpha",  kMaxOverlays);
    auto* activeU = new osg::Uniform(osg::Uniform::INT,        "u_overlayActive", kMaxOverlays);
    for (int i = 0; i < kMaxOverlays; ++i) {
        texU->setElement(i, i);
        transU->setElement(i, osg::Vec2f(0.0f, 0.0f));
        scaleU->setElement(i, osg::Vec2f(1.0f, 1.0f));
        alphaU->setElement(i, 1.0f);
        activeU->setElement(i, 0);
    }
    ss->addUniform(texU);
    ss->addUniform(transU);
    ss->addUniform(scaleU);
    ss->addUniform(alphaU);
    ss->addUniform(activeU);

    // MapNode / OverlayDecorator 등 상위에서 GL_BLEND 가 끄거나 BlendFunc 가 바뀌어도
    // 지형 타일 단위로 표준 알파 블렌드를 유지 (후속 패스·디버그와 일관).
    // 주: 현재 FS는 알파 합성을 RGB 쪽에서 처리하고 frag.a=1 이지만,
    //     확장(출력 알파·다중 패스) 시 부모가 블렌드를 끈 경우를 막기 위함.
    auto* blendFn = new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
    ss->setAttributeAndModes(
        blendFn,
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    ss->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    osgEarth::Lighting::installDefaultMaterial(ss);
    ss->getOrCreateUniform("u_tmsImageryExposure", osg::Uniform::FLOAT)->set(
        HyperTerrainImageryFactory::tmsImageryExposure());
    ss->getOrCreateUniform("oe_sky_ambientBoostFactor", osg::Uniform::FLOAT)->set(1.0f);
}

} // anonymous namespace

HyperTerrainPrepareRendererResources::HyperTerrainPrepareRendererResources(osg::Group* sceneRoot)
    : m_sceneRoot(sceneRoot)
{}

HyperTerrainTileRenderHandle* HyperTerrainPrepareRendererResources::allocateHandle(const HyperTerrainTileRenderData& data) {
    std::lock_guard<std::mutex> lock(m_storeMutex);
    ++m_tickCounter;

    uint32_t slot = 0;
    if (!m_freeSlots.empty()) {
        slot = m_freeSlots.back();
        m_freeSlots.pop_back();
    } else {
        slot = static_cast<uint32_t>(m_store.size());
        m_store.emplace_back();
    }

    TileRenderRecord& record = m_store[slot];
    record.data = data;
    record.alive = true;
    record.createdTick = m_tickCounter;
    record.lastAccessTick = m_tickCounter;

    auto handleStorage = std::make_unique<HyperTerrainTileRenderHandle>();
    HyperTerrainTileRenderHandle* handle = handleStorage.get();
    handle->slot = slot;
    handle->generation = record.generation;
    m_handleStorage.push_back(std::move(handleStorage));
    return handle;
}

bool HyperTerrainPrepareRendererResources::resolveRenderResources(
    const void* renderResources,
    HyperTerrainTileRenderData& outData) const {
    return resolveHandle(static_cast<const HyperTerrainTileRenderHandle*>(renderResources), outData);
}

bool HyperTerrainPrepareRendererResources::resolveHandle(
    const HyperTerrainTileRenderHandle* handle,
    HyperTerrainTileRenderData& outData) const {
    if (!handle) return false;

    std::lock_guard<std::mutex> lock(m_storeMutex);
    if (handle->slot >= m_store.size()) return false;

    const TileRenderRecord& record = m_store[handle->slot];
    if (!record.alive || record.generation != handle->generation) {
        if (m_invalidHandleLogs < 16) {
            ++m_invalidHandleLogs;
            OSG_WARN << "[HyperTerrain] Invalid HyperTerrainTileRenderHandle:"
                     << " slot=" << handle->slot
                     << " gen=" << handle->generation
                     << " recordGen=" << record.generation
                     << " alive=" << record.alive
                     << std::endl;
        }
        return false;
    }

    outData = record.data;
    m_store[handle->slot].lastAccessTick = ++m_tickCounter;
    return true;
}

void HyperTerrainPrepareRendererResources::releaseHandle(const HyperTerrainTileRenderHandle* handle) {
    if (!handle) return;

    std::lock_guard<std::mutex> lock(m_storeMutex);
    if (handle->slot >= m_store.size()) return;

    TileRenderRecord& record = m_store[handle->slot];
    if (!record.alive || record.generation != handle->generation) return;

    record.alive = false;
    record.data = HyperTerrainTileRenderData{};
    ++record.generation;
    m_freeSlots.push_back(handle->slot);
}

bool HyperTerrainPrepareRendererResources::tryResolveTileRenderData(
    const Cesium3DTilesSelection::Tile& tile,
    HyperTerrainTileRenderData& outData) const {
    const auto* rc = tile.getContent().getRenderContent();
    if (!rc) return false;
    return resolveRenderResources(rc->getRenderResources(), outData);
}

float HyperTerrainPrepareRendererResources::rasterOverlayAlpha(
    const CesiumRasterOverlays::RasterOverlay* overlay) const
{
    if (!overlay) return 1.0f;
    std::lock_guard<std::mutex> lock(m_overlayMutex);
    const auto it = m_overlayAlpha.find(overlay);
    return it != m_overlayAlpha.end() ? it->second : 1.0f;
}

void HyperTerrainPrepareRendererResources::setRasterOverlayAlpha(
    const CesiumRasterOverlays::RasterOverlay* overlay,
    float alpha)
{
    if (!overlay) return;

    alpha = std::clamp(alpha, 0.0f, 1.0f);
    std::lock_guard<std::mutex> lock(m_overlayMutex);
    m_overlayAlpha[overlay] = alpha;

    for (RasterAttachment& attachment : m_rasterAttachments) {
        if (attachment.overlay != overlay || !attachment.stateSet.valid() || attachment.slot < 0)
            continue;
        if (auto* u = attachment.stateSet->getUniform("u_overlayAlpha"))
            u->setElement(attachment.slot, alpha);
    }
}

void HyperTerrainPrepareRendererResources::setTmsImageryExposure(float exposure)
{
    exposure = std::max(0.0f, exposure);
    HyperTerrainImageryFactory::setTmsImageryExposure(exposure);

    std::lock_guard<std::mutex> lock(m_storeMutex);
    for (TileRenderRecord& record : m_store) {
        if (!record.alive || !record.data.geom.valid())
            continue;
        osg::StateSet* const ss = record.data.geom->getStateSet();
        if (!ss)
            continue;
        if (osg::Uniform* u = ss->getUniform("u_tmsImageryExposure"))
            u->set(exposure);
        else
            ss->getOrCreateUniform("u_tmsImageryExposure", osg::Uniform::FLOAT)->set(exposure);
    }
}

CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
HyperTerrainPrepareRendererResources::prepareInLoadThread(
    const CesiumAsync::AsyncSystem& asyncSystem,
    Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
    const glm::dmat4& transform,
    const std::any& /*rendererOptions*/)
{
    // 워커 스레드: transform을 쫭처해 main thread에 전달
    auto* ltData = new LoadThreadData();
    ltData->transform = transform;
    return asyncSystem.createResolvedFuture(
        Cesium3DTilesSelection::TileLoadResultAndRenderResources{
            std::move(tileLoadResult), ltData });
}

void* HyperTerrainPrepareRendererResources::prepareInMainThread(
    Cesium3DTilesSelection::Tile& tile,
    void* pLoadThreadResult)
{
    const auto* pContent = tile.getContent().getRenderContent();
    if (!pContent) {
        OSG_WARN << "[HyperTerrain] prepareInMainThread: no render content!" << std::endl;
        if (pLoadThreadResult) delete static_cast<LoadThreadData*>(pLoadThreadResult);
        return nullptr;
    }

    const CesiumGltf::Model& model = pContent->getModel();

    // ---- POSITION → Vec3Array ----
    if (model.meshes.empty() || model.meshes[0].primitives.empty()) {
        if (pLoadThreadResult) delete static_cast<LoadThreadData*>(pLoadThreadResult);
        return nullptr;
    }
    const auto& prim = model.meshes[0].primitives[0];
    auto posIt = prim.attributes.find("POSITION");
    if (posIt == prim.attributes.end() || posIt->second < 0) {
        if (pLoadThreadResult) delete static_cast<LoadThreadData*>(pLoadThreadResult);
        return nullptr;
    }
    const auto& posAcc = model.accessors[static_cast<size_t>(posIt->second)];
    CesiumGltf::AccessorView<glm::vec3> positions(model, posAcc);
    if (positions.status() != CesiumGltf::AccessorViewStatus::Valid || positions.size() == 0) {
        if (pLoadThreadResult) delete static_cast<LoadThreadData*>(pLoadThreadResult);
        return nullptr;
    }

    auto* vtx = new osg::Vec3Array(static_cast<size_t>(positions.size()));
    for (int64_t i = 0; i < positions.size(); ++i)
        (*vtx)[static_cast<size_t>(i)].set(positions[i].x, positions[i].y, positions[i].z);

    // ---- 법선 계산 (glTF NORMAL accessor 또는 삼각형에서 계산) ----
    osg::Vec3Array* nrm = buildNormals(model, prim, positions);

    // ---- 인덱스 버퍼 ----
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    geom->setUseDisplayList(false);
    geom->setUseVertexBufferObjects(true);
    geom->setVertexAttribArray(0, vtx, osg::Array::BIND_PER_VERTEX);
    geom->setVertexAttribArray(1, nrm, osg::Array::BIND_PER_VERTEX);

    // ---- _CESIUMOVERLAY_0..3 for raster overlay UV (attribute location 2..5) ----
    // cesium-native는 오버레이 UV를 TEXCOORD_N이 아닌 _CESIUMOVERLAY_N 이름으로 저장한다.
    for (int tc = 0; tc < kMaxOverlays; ++tc) {
        const std::string name = std::string("_CESIUMOVERLAY_") + std::to_string(tc);
        auto tcIt = prim.attributes.find(name);
        if (tcIt != prim.attributes.end() && tcIt->second >= 0
            && static_cast<size_t>(tcIt->second) < model.accessors.size()) {
            CesiumGltf::AccessorView<glm::vec2> tcView(
                model, model.accessors[static_cast<size_t>(tcIt->second)]);
            if (tcView.status() == CesiumGltf::AccessorViewStatus::Valid && tcView.size() > 0) {
                auto* tcArr = new osg::Vec2Array(static_cast<size_t>(tcView.size()));
                for (int64_t i = 0; i < tcView.size(); ++i)
                    (*tcArr)[static_cast<size_t>(i)].set(tcView[i].x, tcView[i].y);
                geom->setVertexAttribArray(2 + tc, tcArr, osg::Array::BIND_PER_VERTEX);
            }
        }
    }

    // 동일 투영(Web Mercator 등)의 오버레이는 glTF에 _CESIUMOVERLAY_0 만 있고 ID는 둘 다 0이다.
    // 두 번째 오버레이를 OSG 유닛 1..3에 올릴 때 동일 Mercator UV가 필요하므로, 첫 유효 슬롯 배열을 공유한다.
    {
        osg::Vec2Array* sharedMercatorUvs = nullptr;
        for (int tc = 0; tc < kMaxOverlays; ++tc) {
            osg::Array* arr = geom->getVertexAttribArray(2 + tc);
            if (arr) {
                sharedMercatorUvs = dynamic_cast<osg::Vec2Array*>(arr);
                if (sharedMercatorUvs)
                    break;
            }
        }
        if (sharedMercatorUvs) {
            for (int tc = 0; tc < kMaxOverlays; ++tc) {
                if (!geom->getVertexAttribArray(2 + tc)) {
                    geom->setVertexAttribArray(2 + tc, sharedMercatorUvs, osg::Array::BIND_PER_VERTEX);
                }
            }
        }
    }

    if (prim.indices >= 0 && static_cast<size_t>(prim.indices) < model.accessors.size()) {
        const auto& idxAcc = model.accessors[static_cast<size_t>(prim.indices)];
        CesiumGltf::AccessorView<uint16_t> idx16(model, idxAcc);
        if (idx16.status() == CesiumGltf::AccessorViewStatus::Valid) {
            auto* de = new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLES, static_cast<size_t>(idx16.size()));
            for (int64_t i = 0; i < idx16.size(); ++i) (*de)[static_cast<size_t>(i)] = idx16[i];
            geom->addPrimitiveSet(de);
        } else {
            CesiumGltf::AccessorView<uint32_t> idx32(model, idxAcc);
            if (idx32.status() == CesiumGltf::AccessorViewStatus::Valid) {
                auto* de = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, static_cast<size_t>(idx32.size()));
                for (int64_t i = 0; i < idx32.size(); ++i) (*de)[static_cast<size_t>(i)] = idx32[i];
                geom->addPrimitiveSet(de);
            }
        }
    } else {
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, static_cast<int>(positions.size())));
    }

    // ---- 셰이더 설정 ----
    osg::StateSet* ss = geom->getOrCreateStateSet();
    // Keep terrain material local so clamping RTT/depth passes can still install
    // their own state as needed.
    ss->setAttribute(
        buildSimpleShader(),
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    // 뒷면 컬링 비활성화 (법선 방향 상관없이 렌더)
    ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
    // depth test 명시적 활성화
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    // Clampable overlays are rendered at a much higher bin (200000). Keep terrain explicit.
    ss->setRenderBinDetails(0, "RenderBin");
    // 오버레이 유니폼 기본값 초기화 (attach 전까지 비활성)
    initOverlayUniforms(ss);

    auto* geode = new osg::Geode();
    geode->addDrawable(geom);

    // ---- MatrixTransform: prepareInLoadThread transform * glTF node matrix ----
    glm::dmat4 worldTransform(1.0);
    if (pLoadThreadResult) {
        worldTransform = static_cast<const LoadThreadData*>(pLoadThreadResult)->transform;
    }
    if (!model.nodes.empty()) {
        const auto& node = model.nodes[0];
        if (node.matrix.size() == 16) {
            // glTF는 column-major 저장: matrix[0..3]=col0, matrix[4..7]=col1, ..., matrix[12..15]=col3(translation)
            // glm::dmat4(scalar...) 생성자: 첫 4개 = col0, 다음 4개 = col1, ...  (column-major)
            glm::dmat4 nodeMat(
                node.matrix[0],  node.matrix[1],  node.matrix[2],  node.matrix[3],
                node.matrix[4],  node.matrix[5],  node.matrix[6],  node.matrix[7],
                node.matrix[8],  node.matrix[9],  node.matrix[10], node.matrix[11],
                node.matrix[12], node.matrix[13], node.matrix[14], node.matrix[15]);
            worldTransform = worldTransform * nodeMat;
        } else if (node.translation.size() == 3) {
            // matrix 없이 TRS decomposition
            worldTransform[3] = glm::dvec4(node.translation[0], node.translation[1], node.translation[2], 1.0);
        }
    }

    // ---- glTF Y-up → ECEF Z-up 변환 ----
    // cesium-native glTF 타일은 Y-up 월드 공간 기준:
    //   X_ecef = X_world
    //   Y_ecef = -Z_world
    //   Z_ecef = Y_world
    // glm M*v column-major: col0=(1,0,0,0), col1=(0,0,1,0), col2=(0,-1,0,0), col3=(0,0,0,1)
    static const glm::dmat4 kYUpToZUp(
        1.0,  0.0, 0.0, 0.0,  // col0
        0.0,  0.0, 1.0, 0.0,  // col1
        0.0, -1.0, 0.0, 0.0,  // col2
        0.0,  0.0, 0.0, 1.0); // col3
    worldTransform = kYUpToZUp * worldTransform;

    // ---- glm::dmat4 → osg::Matrixd 변환 ----
    // glm: column-major 저장, M*v (열벡터)
    // OSG: row-major 저장, v*M (행벡터)
    // 같은 16개 더블 메모리를 그대로 쓰면 수학적으로 동등한 변환 → glm::value_ptr 사용
    const double* p = glm::value_ptr(worldTransform);
    osg::Matrixd osgMat(
        p[0],  p[1],  p[2],  p[3],
        p[4],  p[5],  p[6],  p[7],
        p[8],  p[9],  p[10], p[11],
        p[12], p[13], p[14], p[15]);

    auto* xformNode = new osg::MatrixTransform(osgMat);
    xformNode->addChild(geode);

    if (m_sceneRoot.valid()) {
        m_sceneRoot->addChild(xformNode);
    }

    HyperTerrainTileRenderData data;
    data.xform = xformNode;
    data.geom = geom;
    HyperTerrainTileRenderHandle* handle = allocateHandle(data);
    if (pLoadThreadResult) delete static_cast<LoadThreadData*>(pLoadThreadResult);
    return handle;
}

void HyperTerrainPrepareRendererResources::free(
    Cesium3DTilesSelection::Tile& /*tile*/,
    void* pLoadThreadResult,
    void* pMainThreadResult) noexcept
{
    if (pLoadThreadResult) {
        delete static_cast<LoadThreadData*>(pLoadThreadResult);
    }

    if (!pMainThreadResult) return;
    auto* handle = static_cast<HyperTerrainTileRenderHandle*>(pMainThreadResult);
    HyperTerrainTileRenderData data;
    const bool resolved = resolveHandle(handle, data);

    if (resolved && m_sceneRoot.valid() && data.xform.valid()) {
        m_sceneRoot->removeChild(data.xform);
    }
    if (resolved && data.geom.valid()) {
        osg::StateSet* ss = data.geom->getStateSet();
        std::lock_guard<std::mutex> lock(m_overlayMutex);
        m_rasterAttachments.erase(
            std::remove_if(
                m_rasterAttachments.begin(),
                m_rasterAttachments.end(),
                [&](const RasterAttachment& attachment) {
                    return attachment.stateSet.get() == ss;
                }),
            m_rasterAttachments.end());
    }

    releaseHandle(handle);
}

// ---- RasterOverlay 위임 ----

void* HyperTerrainPrepareRendererResources::prepareRasterInLoadThread(
    CesiumGltf::ImageAsset& image,
    const std::any& rendererOptions)
{
    HyperTerrainRasterRenderer rasterRenderer;
    return rasterRenderer.prepareRasterInLoadThread(image, rendererOptions);
}

void* HyperTerrainPrepareRendererResources::prepareRasterInMainThread(
    CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pLoadThreadResult)
{
    HyperTerrainRasterRenderer rasterRenderer;
    return rasterRenderer.prepareRasterInMainThread(rasterTile, pLoadThreadResult);
}

void HyperTerrainPrepareRendererResources::freeRaster(
    const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pLoadThreadResult,
    void* pMainThreadResult) noexcept
{
    HyperTerrainRasterRenderer rasterRenderer;
    rasterRenderer.freeRaster(rasterTile, pLoadThreadResult, pMainThreadResult);
}

void HyperTerrainPrepareRendererResources::attachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile,
    int32_t overlayTextureCoordinateID,
    const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pMainThreadRendererResources,
    const glm::dvec2& translation,
    const glm::dvec2& scale)
{
    if (!pMainThreadRendererResources) return;
    HyperTerrainTileRenderData tileData;
    if (!tryResolveTileRenderData(tile, tileData) || !tileData.geom.valid()) {
        OSG_WARN << "[HyperTerrain] attachRasterInMainThread: invalid tile render data." << std::endl;
        return;
    }

    const int cesiumTcId = std::max(0, std::min(static_cast<int>(overlayTextureCoordinateID), kMaxOverlays - 1));
    auto* tex = static_cast<osg::Texture2D*>(pMainThreadRendererResources);
    osg::StateSet* ss = tileData.geom->getOrCreateStateSet();

    // 동일 textureCoordinateID(대개 0)를 쓰는 베이스 TMS/위성 + OWM 구름 등이 겹치면
    // 같은 OSG 유닛에 연달아 바인딩되어 먼저 붙은 타일이 사라진다. 빈 유닛으로 분산한다.
    int slot = cesiumTcId;
    auto* existing = dynamic_cast<osg::Texture2D*>(
        ss->getTextureAttribute(cesiumTcId, osg::StateAttribute::TEXTURE));
    if (existing != nullptr && existing != tex) {
        slot = -1;
        for (int j = 0; j < kMaxOverlays; ++j) {
            auto* ej = dynamic_cast<osg::Texture2D*>(ss->getTextureAttribute(j, osg::StateAttribute::TEXTURE));
            if (ej == nullptr) {
                slot = j;
                break;
            }
        }
        if (slot < 0) {
            OSG_WARN << "[HyperTerrain] attachRasterInMainThread: all " << kMaxOverlays
                     << " overlay texture units in use; cannot attach raster." << std::endl;
            return;
        }
    }

    ss->setTextureAttributeAndModes(slot, tex, osg::StateAttribute::ON);
    if (auto* u = ss->getUniform("u_overlayTex"))    u->setElement(slot, slot);
    if (auto* u = ss->getUniform("u_overlayTrans"))   u->setElement(slot,
        osg::Vec2f(static_cast<float>(translation.x), static_cast<float>(translation.y)));
    if (auto* u = ss->getUniform("u_overlayScale"))   u->setElement(slot,
        osg::Vec2f(static_cast<float>(scale.x), static_cast<float>(scale.y)));
    if (auto* u = ss->getUniform("u_overlayAlpha"))   u->setElement(slot,
        rasterOverlayAlpha(&rasterTile.getOverlay()));
    if (auto* u = ss->getUniform("u_overlayActive"))  u->setElement(slot, 1);

    {
        std::lock_guard<std::mutex> lock(m_overlayMutex);
        m_rasterAttachments.push_back(
            RasterAttachment{&rasterTile.getOverlay(), ss, tex, slot});
    }
}

void HyperTerrainPrepareRendererResources::detachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile,
    int32_t overlayTextureCoordinateID,
    const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pMainThreadRendererResources) noexcept
{
    HyperTerrainTileRenderData tileData;
    if (!tryResolveTileRenderData(tile, tileData) || !tileData.geom.valid()) return;

    osg::StateSet* ss = tileData.geom->getOrCreateStateSet();

    if (pMainThreadRendererResources) {
        auto* tex = static_cast<osg::Texture2D*>(pMainThreadRendererResources);
        for (int s = 0; s < kMaxOverlays; ++s) {
            auto* bound = dynamic_cast<osg::Texture2D*>(
                ss->getTextureAttribute(s, osg::StateAttribute::TEXTURE));
            if (bound == tex) {
                ss->removeTextureAttribute(s, osg::StateAttribute::TEXTURE);
                if (auto* u = ss->getUniform("u_overlayActive"))
                    u->setElement(s, 0);
                {
                    std::lock_guard<std::mutex> lock(m_overlayMutex);
                    m_rasterAttachments.erase(
                        std::remove_if(
                            m_rasterAttachments.begin(),
                            m_rasterAttachments.end(),
                            [&](const RasterAttachment& attachment) {
                                return attachment.overlay == &rasterTile.getOverlay()
                                    && attachment.stateSet.get() == ss
                                    && attachment.slot == s;
                            }),
                        m_rasterAttachments.end());
                }
                return;
            }
        }
    }

    const int slot = std::min(static_cast<int>(overlayTextureCoordinateID), kMaxOverlays - 1);
    ss->removeTextureAttribute(slot, osg::StateAttribute::TEXTURE);
    if (auto* u = ss->getUniform("u_overlayActive"))  u->setElement(slot, 0);
    {
        std::lock_guard<std::mutex> lock(m_overlayMutex);
        m_rasterAttachments.erase(
            std::remove_if(
                m_rasterAttachments.begin(),
                m_rasterAttachments.end(),
                [&](const RasterAttachment& attachment) {
                    return attachment.overlay == &rasterTile.getOverlay()
                        && attachment.stateSet.get() == ss
                        && attachment.slot == slot;
                }),
            m_rasterAttachments.end());
    }
}

} // namespace Cesium
} // namespace osgEarth
