// note: this is an include file

// depth texture captures by the clamping technique
uniform sampler2D oe_clamp_depthTex;

// matrix transforming from view space to depth-texture clip space
uniform mat4 oe_clamp_cameraView2depthClip;

// matrix transform from depth-tecture clip space to view space
uniform mat4 oe_clamp_depthClip2cameraView;

// Given a vertex in view space, clamp it to the "ground" as represented
// by an orthographic depth texture. Return the clamped vertex in view space,
// along with the associated depth value.
void oe_getClampedViewVertex(in vec4 vertView, out vec4 out_clampedVertView, out float out_depth)
{
    vec4 vertDepthClip = oe_clamp_cameraView2depthClip * vertView;

    vec2 uv = vertDepthClip.xy / vertDepthClip.w;
    const float margin = 0.005;
    vec2 safeUV = clamp(uv, vec2(margin), vec2(1.0 - margin));

    // Sample at a clamped UV so the depth is always consistent with the
    // reconstructed position — prevents the x,y / z mismatch that causes
    // huge vertical offsets when the vertex projects outside the depth texture.
    out_depth = texture(oe_clamp_depthTex, safeUV).r;

    vec4 clampedVertDepthClip = vec4(
        safeUV.x * vertDepthClip.w,
        safeUV.y * vertDepthClip.w,
        out_depth, 1.0);

    out_clampedVertView = oe_clamp_depthClip2cameraView * clampedVertDepthClip;
}

// Returns a vector indicating the "down" direction.
void oe_getClampingUpVector(out vec3 up)
{
    up = normalize(mat3(oe_clamp_depthClip2cameraView) * vec3(0,0,-1));
}
