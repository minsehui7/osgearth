/* osgEarth Cesium - shared Ion/HyperTerrain lit mesh shaders */
#include "HyperTerrainLitShader"
#include "HyperTerrainImageryFactory"

#include <osg/BlendFunc>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Matrixf>
#include <osg/Uniform>

#include <osgEarth/Lighting>

#include <string>

namespace osgEarth { namespace Cesium {

namespace {

constexpr int kMaxOverlays = 4;

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
out vec3 v_worldPos;

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

    vec4 worldPos4 = osg_ViewMatrixInverse * vertexVIEW;
    v_worldPos = worldPos4.xyz / max(worldPos4.w, 1e-6);

    // 거리 헤이즈 인자: ECEF world-space 좌표 기준으로 hl_fog_center 까지의 거리.
    // hl_fog_end <= 0 이면 비활성 (ViewOsgEarth.cpp::installDistanceFog 초기값).
    if (hl_fog_end > 0.0) {
        float dist = length(v_worldPos - hl_fog_center);
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

uniform int   hl_clip_enabled;
uniform int   hl_clip_union;
uniform int   hl_clip_plane_count;
uniform mat4  hl_clip_modelMatrixInverse;
uniform vec4  hl_clip_planes[64];

in vec3 v_worldPos;

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

// Cesium ClippingPlaneCollection (intersection): keep where dot(n,p)+d <= 0 for all planes.
bool hyperTerrainClipDiscard(vec3 worldPos) {
    if (hl_clip_enabled == 0 || hl_clip_plane_count <= 0)
        return false;
    vec3 localPos = (hl_clip_modelMatrixInverse * vec4(worldPos, 1.0)).xyz;
    if (hl_clip_union != 0) {
        for (int i = 0; i < hl_clip_plane_count; ++i) {
            vec4 pl = hl_clip_planes[i];
            if (dot(pl.xyz, localPos) + pl.w <= 0.0)
                return true;
        }
        return false;
    }
    for (int i = 0; i < hl_clip_plane_count; ++i) {
        vec4 pl = hl_clip_planes[i];
        if (dot(pl.xyz, localPos) + pl.w > 0.0)
            return true;
    }
    return false;
}

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
    if (hyperTerrainClipDiscard(v_worldPos))
        discard;

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

const char* kVertAlbedoDecl =
    "#ifdef HYPER_TERRAIN_VERTEX_ALBEDO\n"
    "layout(location = 6) in vec3 a_albedo;\n"
    "out vec3 v_albedo;\n"
    "#endif\n";

const char* kVertAlbedoMain =
    "#ifdef HYPER_TERRAIN_VERTEX_ALBEDO\n"
    "    v_albedo = a_albedo;\n"
    "#endif\n";

const char* kFragAlbedoDecl =
    "#ifdef HYPER_TERRAIN_VERTEX_ALBEDO\n"
    "in vec3 v_albedo;\n"
    "#endif\n";

static std::string injectAfterVersion(const char* src, const char* snippet) {
    std::string out(src);
    const size_t pos = out.find("#version");
    if (pos != std::string::npos && snippet && snippet[0]) {
        const size_t lineEnd = out.find('\n', pos);
        const size_t insertAt = (lineEnd != std::string::npos) ? lineEnd + 1 : pos;
        out.insert(insertAt, snippet);
    }
    return out;
}

static std::string injectBeforeMain(const char* src, const char* snippet) {
    std::string out(src);
    const std::string needle("void main()");
    const size_t pos = out.rfind(needle);
    if (pos != std::string::npos && snippet && snippet[0])
        out.insert(pos, snippet);
    return out;
}

static std::string patchFragBaseColor(const std::string& frag) {
    const std::string oldLine("    vec3 colorRgb = kBaseColor;");
    const size_t pos = frag.find(oldLine);
    if (pos == std::string::npos)
        return frag;
    std::string out = frag;
    out.replace(pos, oldLine.size(), "    vec3 colorRgb = HYPER_TERRAIN_VERTEX_ALBEDO ? v_albedo : kBaseColor;");
    return out;
}

static osg::ref_ptr<osg::Program> buildProgram(HyperTerrainLitColorMode colorMode) {
    std::string vertSrc(kSimpleVertGLSL);
    std::string fragSrc(kSimpleFragGLSL);
    if (colorMode == HyperTerrainLitColorMode::PerVertexAlbedo) {
        vertSrc = injectAfterVersion(vertSrc.c_str(), kVertAlbedoDecl);
        vertSrc = injectBeforeMain(vertSrc.c_str(), kVertAlbedoMain);
        fragSrc = injectAfterVersion(fragSrc.c_str(), kFragAlbedoDecl);
        fragSrc = patchFragBaseColor(fragSrc);
        fragSrc = injectAfterVersion(fragSrc.c_str(), "#define HYPER_TERRAIN_VERTEX_ALBEDO 1\n");
    }
    osg::ref_ptr<osg::Program> prog = new osg::Program();
    prog->setName(colorMode == HyperTerrainLitColorMode::PerVertexAlbedo
        ? "HyperTerrainLitAtmosphereVertexAlbedo" : "HyperTerrainLitAtmosphere");
    prog->addShader(new osg::Shader(osg::Shader::VERTEX, vertSrc.c_str()));
    prog->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSrc.c_str()));
    prog->addBindAttribLocation("a_position", 0);
    prog->addBindAttribLocation("a_normal", 1);
    prog->addBindAttribLocation("a_texcoord0", 2);
    prog->addBindAttribLocation("a_texcoord1", 3);
    prog->addBindAttribLocation("a_texcoord2", 4);
    prog->addBindAttribLocation("a_texcoord3", 5);
    if (colorMode == HyperTerrainLitColorMode::PerVertexAlbedo)
        prog->addBindAttribLocation("a_albedo", 6);
    return prog;
}

} // namespace

constexpr int kHyperTerrainMaxOverlays = 4;

osg::ref_ptr<osg::Program> createHyperTerrainLitProgram(HyperTerrainLitColorMode colorMode) {
    return buildProgram(colorMode);
}

void initHyperTerrainLitStateSet(osg::StateSet* ss) {
    if (!ss)
        return;
    auto* texU    = new osg::Uniform(osg::Uniform::SAMPLER_2D, "u_overlayTex",    kHyperTerrainMaxOverlays);
    auto* transU  = new osg::Uniform(osg::Uniform::FLOAT_VEC2, "u_overlayTrans",  kHyperTerrainMaxOverlays);
    auto* scaleU  = new osg::Uniform(osg::Uniform::FLOAT_VEC2, "u_overlayScale",  kHyperTerrainMaxOverlays);
    auto* alphaU  = new osg::Uniform(osg::Uniform::FLOAT,      "u_overlayAlpha",  kHyperTerrainMaxOverlays);
    auto* activeU = new osg::Uniform(osg::Uniform::INT,        "u_overlayActive", kHyperTerrainMaxOverlays);
    for (int i = 0; i < kHyperTerrainMaxOverlays; ++i) {
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
    // Do not set oe_overlay_ready / oe_overlay_texmatrix here: per-tile defaults (ready=0)
    // override DrapingTechnique uniforms from OverlayDecorator::_sharedTerrainStateSet during
    // draw and disable applyDrapeOverlay() even when the drape RTT is valid.
    auto* blendFn = new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
    ss->setAttributeAndModes(blendFn, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    ss->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    osgEarth::Lighting::installDefaultMaterial(ss);
    ss->getOrCreateUniform("u_tmsImageryExposure", osg::Uniform::FLOAT)->set(
        HyperTerrainImageryFactory::tmsImageryExposure());
    ss->getOrCreateUniform("oe_sky_ambientBoostFactor", osg::Uniform::FLOAT)->set(1.0f);
}

}} // namespace osgEarth::Cesium
