#version 330 core

uniform struct LightInfo {
    vec4 position;
    vec3 intensity;
} light;

struct MaterialInfo
{
    vec3 ka;            // Ambient reflectivity
    vec3 kd;            // Diffuse reflectivity
    vec3 ks;            // Specular reflectivity
    float shininess;    // Specular shininess factor
};
uniform MaterialInfo material;

in vec3 position;
in vec3 normal;

out vec4 fragColor;


vec3 adsModel( const in vec3 pos, const in vec3 n )
{
    vec3 s = normalize( vec3( light.position ) - pos );
    vec3 v = normalize( -pos );
    vec3 r = reflect( -s, n );

    float diffuse = max( dot( s, n ), 0.0 );

    float specular = 0.0;
    if ( dot( s, n ) > 0.0 )
        specular = pow( max( dot( r, v ), 0.0 ), material.shininess );

    return light.intensity * ( material.ka + material.kd * diffuse + material.ks * specular );
}

void main()
{
    fragColor = vec4( adsModel( position, normalize( normal ) ), 1.0 );
}
