// The PS VR2 compositor: for every pixel of the 4000x2040 panel, where it looks through the lens
// (the headset's own calibration and Monado's table, as psvr2-player-mac corrects it), turned by
// the head's rotation since the game rendered, and that ray's place in the game's eye image.
// With quad views the game's sharp inset is laid over the rest, faded in at its edges. The eye
// tracker's calibration dots and the gaze ring (backend_psvr2.m) are drawn here too.
#include <metal_stdlib>
using namespace metal;

struct VOut { float4 pos [[position]]; float2 uv; };

vertex VOut vs(uint vid [[vertex_id]]) {
    float2 p[3] = { float2(-1.0, -1.0), float2(3.0, -1.0), float2(-1.0, 3.0) };
    VOut o;
    o.pos = float4(p[vid], 0.0, 1.0);
    o.uv = float2(p[vid].x * 0.5 + 0.5, 1.0 - (p[vid].y * 0.5 + 0.5));   // (0,0) top left
    return o;
}

struct Uni {
    float4 calibL;     // k1, k2, k3, k4 of each eye, from the headset (psvr2_get_distortion_calibration)
    float4 calibR;
    float4x4 delta;    // a direction in the head now -> in the head the game rendered for
    float4 p0;         // x: a game frame is there, y: chromatic correction, z: quad insets, w: gaze ring
    float4 p1;         // passthrough: x: a camera frame is there, y: lens angle (rad), z: brightness,
                       // w: convergence (fractions of the frame)
    float4 cal;        // calibration target in the head (m); w: 0 off, 1 dark, 2 light
    float4 cal2;       // x: half the IPD (m), y: seconds since the target appeared
    float4 gaze;       // where the eyes look, unit vector in the head
    float4 insetL;     // tangents of each inset's field of view: left, right, up, down
    float4 insetR;
};

// The game's eye images cover this much of tan space: x from -cx / FX to (1 - cx) / FX, y from
// -0.5 / FY to 0.5 / FY. The runtime reports the same field of view (backend_psvr2.m).
constant float FX = 0.3585564;
constant float FY = 0.3762281;

fragment float4 fs(VOut in [[stage_in]], constant Uni &u [[buffer(0)]],
                   device const packed_float3 *lut [[buffer(1)]],
                   texture2d<float> left [[texture(0)]], texture2d<float> right [[texture(1)]],
                   texture2d<float> camL [[texture(2)]], texture2d<float> camR [[texture(3)]],
                   texture2d<float> insetLeft [[texture(4)]], texture2d<float> insetRight [[texture(5)]]) {
    constexpr sampler s(filter::linear, address::clamp_to_edge);
    int eye = in.uv.x < 0.5 ? 0 : 1;
    float fU = eye == 0 ? in.uv.x * 2.0 : (in.uv.x - 0.5) * 2.0;
    float fV = in.uv.y;

    float4 c = eye == 0 ? u.calibL : u.calibR;
    float k1 = c.x, k2 = c.y, k3 = c.z, k4 = c.w;
    float local_x = k1 + (fU * 2.0 - 1.0) * 0.9803922 * 0.9394987;
    float local_y = k2 + (fV * 2.0 - 1.0) * 0.9394987;

    float Rs = (local_x * local_x + local_y * local_y) * 1023.0;
    int idx = int(Rs);
    float3 scale;
    if (idx > -1 && idx + 1 < 1024) {
        float frac = Rs - float(idx);
        scale = frac * float3(lut[idx + 1]) + (1.0 - frac) * float3(lut[idx]);
    } else {
        float v = Rs - 1023.0;
        scale = float3(v * 0.003648639 + 1.592586, v * 0.004862547 + 1.692979, v * 0.007425308 + 1.841771);
    }

    // The ray through this pixel in the head now (green's lens geometry), for what is drawn
    // fixed to the head: calibration dots and the gaze ring.
    float ga = local_x * scale[1], gb = (local_y - 0.0002302693) * scale[1];
    float3 ray = normalize(float3(k3 * ga - k4 * gb, -(k4 * ga + k3 * gb), -1.0));

    if (u.cal.w > 0.5) {
        // A dot with a ring that closes in on it over a second, seen from this eye.
        float3 to = normalize(u.cal.xyz - float3(eye == 0 ? -u.cal2.x : u.cal2.x, 0.0, 0.0));
        float ang = acos(clamp(dot(ray, to), -1.0, 1.0));
        bool light = u.cal.w > 1.5;
        float3 bg = light ? float3(0.7) : float3(0.0), fg = light ? float3(0.0) : float3(1.0);
        float core = 1.0 - smoothstep(0.004, 0.006, ang);
        float ring = (1.0 - smoothstep(0.0015, 0.003, abs(ang - mix(0.06, 0.012, saturate(u.cal2.y))))) * 0.8;
        return float4(mix(bg, fg, max(core, ring)), 1.0);
    }

    // Passthrough: the headset's front cameras in place of the game, as psvr2-player-mac shows
    // them. They move with the head, so no head turn applies: an equidistant fisheye straight
    // ahead, each eye from its own camera, shifted toward the other since the cameras sit wider
    // apart than the eyes. The picture is grayscale (the cameras are monochrome).
    if (u.p1.x > 0.5) {
        float a = local_x * scale[1];
        float b = (local_y - 0.0002302693) * scale[1];
        float3 dir = normalize(float3(k3 * a - k4 * b, -(k4 * a + k3 * b), -1.0));
        float theta = acos(clamp(-dir.z, -1.0, 1.0));
        float r = theta / u.p1.y;
        if (r > 0.5) { return float4(0.0, 0.0, 0.0, 1.0); }
        float len = length(dir.xy);
        float2 dd = len > 1e-6 ? dir.xy / len : float2(0.0);
        float shift = eye == 0 ? u.p1.w : -u.p1.w;
        float2 cuv = float2((0.5 + r * dd.x + shift) * (1016.0 / 1024.0), 0.5 - r * dd.y);   // 1016 wide in 1024
        float g = saturate((eye == 0 ? camL : camR).sample(s, cuv).r * u.p1.z);
        // The player writes this value to a plain BGRA panel; this one is sRGB, so undo its encoding
        float lin = g <= 0.04045 ? g / 12.92 : pow((g + 0.055) / 1.055, 2.4);
        return float4(float3(lin), 1.0);
    }

    if (u.p0.x < 0.5) { return float4(0.08, 0.08, 0.08, 1.0); }   // no game frame yet

    float cx = eye == 0 ? 0.6603788 : 0.3396213;
    bool chromatic = u.p0.y > 0.5;
    int from = chromatic ? 0 : 1, to = chromatic ? 2 : 1;
    float3 rgb = float3(0.0);
    for (int ch = from; ch <= to; ch++) {
        float a = local_x * scale[ch];
        float b = (local_y + (ch == 1 ? -0.0002302693 : 0.0)) * scale[ch];
        float tanx = k3 * a - k4 * b;
        float tanyDown = k4 * a + k3 * b;
        float3 d = (u.delta * float4(tanx, -tanyDown, -1.0, 0.0)).xyz;
        if (d.z > -1e-3) { continue; }
        float2 uv = float2(d.x / -d.z * FX + cx, -d.y / -d.z * FY + 0.5);
        if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) { continue; }
        float3 sampled = (eye == 0 ? left.sample(s, uv) : right.sample(s, uv)).rgb;
        if (u.p0.z > 0.5) {
            float4 r = eye == 0 ? u.insetL : u.insetR;
            float2 iuv = float2((d.x / -d.z - r.x) / (r.y - r.x), (r.z - d.y / -d.z) / (r.z - r.w));
            float2 edge = min(iuv, 1.0 - iuv);
            float w = smoothstep(0.0, 0.08, min(edge.x, edge.y));
            if (w > 0.0) {
                float3 inset = (eye == 0 ? insetLeft.sample(s, iuv) : insetRight.sample(s, iuv)).rgb;
                sampled = mix(sampled, inset, w);
            }
        }
        if (chromatic) { rgb[ch] = sampled[ch]; } else { rgb = sampled; }
    }
    if (u.p0.w > 0.5) {
        // A yellow ring about 1.5 degrees across where the eyes look.
        float ang = acos(clamp(dot(ray, u.gaze.xyz), -1.0, 1.0));
        float ring = 1.0 - smoothstep(0.002, 0.004, abs(ang - 0.013));
        rgb = mix(rgb, float3(1.0, 0.85, 0.1), ring);
    }
    return float4(rgb, 1.0);
}
