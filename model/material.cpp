/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "model/material.h"
#include "rendering/renderer.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"
#include "utilities/Logs.h"
#include "scene/sn_utils.h"
#include "utilities/Globals.h"

opengl_material::path_data opengl_material::paths;

opengl_material::opengl_material()
{
    for (size_t i = 0; i < params.size(); i++)
        params[i] = glm::vec4(std::numeric_limits<float>::quiet_NaN());
}

bool
opengl_material::deserialize( cParser &Input, bool const Loadnow ) {
    parse_info = std::make_unique<parse_info_s>();

    bool result { false };
    while( true == deserialize_mapping( Input, 0, Loadnow ) ) {
        result = true; // once would suffice but, eh
    }

    if( path == -1
     && ( update_on_weather_change || update_on_season_change ) ) {
        // record current texture path in the material, potentially needed when material is reloaded on environment change
        // NOTE: we're storing this only for textures that can actually change, to keep the size of path database modest
        auto const lookup{ paths.index_map.find( Global.asCurrentTexturePath ) };
        if( lookup != paths.index_map.end() ) {
            path = lookup->second;
        }
        else {
            path = paths.data.size();
            paths.data.emplace_back( Global.asCurrentTexturePath );
            paths.index_map.emplace( Global.asCurrentTexturePath, path );
        }
    }

    return result;
}

void opengl_material::log_error(const std::string &str)
{
    ErrorLog("bad material: " + name + ": " + str, logtype::material);
}

std::map<std::string, int> texture_bindings {

    { "diffuse", 0 },
    { "normals", 1 },
    { "normalmap", 1 }
};

int opengl_material::texture_slot( std::string const &Key )
{
    auto key { Key };

    if( ( key.size() > 0 ) && ( key[ 0 ] != '_' ) )
    {
        // numbered binding, "textureN:" fills the slot N-1
        key.erase( key.find_first_not_of( "-1234567890" ) );
        size_t const num = std::stoi( key ) - 1;
        if( num < gl::MAX_TEXTURES ) {
            return static_cast<int>( num );
        }
        log_error( "invalid texture binding: " + std::to_string( num ) );
        is_good = false;
        return -1;
    }

    if( key.size() > 2 )
    {
        // named binding, the slot it fills is up to the shader
        key.erase( 0, 1 );
        key.pop_back();
        if( shader && ( shader->texture_conf.find( key ) != shader->texture_conf.end() ) ) {
            return static_cast<int>( shader->texture_conf[ key ].id );
        }
        auto const lookup { texture_bindings.find( key ) };
        if( ( shader == nullptr ) && ( lookup != texture_bindings.end() ) ) {
            return lookup->second;
        }
        // ignore unrecognized texture bindings in legacy render mode, it's most likely data for more advanced shaders
        if( Global.GfxRenderer == "default" ) {
            log_error( "unknown texture binding: " + key );
            is_good = false;
        }
        return -1;
    }

    log_error( "unrecognized texture binding: " + key );
    is_good = false;
    return -1;
}

void opengl_material::build_texture_variants( std::vector<variant_binding> const &Bindings, bool const Loadnow )
{
    if( true == Bindings.empty() ) { return; }

    // build a texture set for each entry of the largest of the declared pools.
    // pools of different size can be combined, shorter ones simply repeat from the start
    std::size_t variantcount { 0 };
    for( auto const &[ slot, pool ] : Bindings ) {
        variantcount = std::max( variantcount, pool->size() );
    }
    texture_variants.resize( variantcount, textures );

    for( auto const &[ slot, pool ] : Bindings ) {
        for( std::size_t variantindex = 0; variantindex < variantcount; ++variantindex ) {
            auto const &texturename { ( *pool )[ variantindex % pool->size() ] };
            auto const texturehandle { GfxRenderer->Fetch_Texture( texturename, Loadnow ) };
            if( texturehandle == null_handle ) {
                // a single bad entry shouldn't take down the whole material, fall back to the basic texture
                log_error( "missing texture variant: " + texturename );
                continue;
            }
            texture_variants[ variantindex ][ slot ] = texturehandle;
        }
    }
}

void opengl_material::finalize(bool Loadnow)
{
    is_good = true;

    if (parse_info)
    {
        // texture bindings which come with a pool of alternatives, gathered while resolving regular bindings
        std::vector<variant_binding> variantbindings;

        for( auto const &[ bindingkey, binding ] : parse_info->tex_mapping )
        {
            auto const slot { texture_slot( bindingkey ) };
            if( slot < 0 ) { continue; }

            textures[ slot ] = GfxRenderer->Fetch_Texture( binding.name, Loadnow );
            if( binding.variants.size() > 1 ) {
                variantbindings.emplace_back( static_cast<std::size_t>( slot ), &( binding.variants ) );
            }
        }

        build_texture_variants( variantbindings, Loadnow );

        if (!shader)
        {
// TODO: add error severity to logging, re-enable these errors as low severity messages
            if (textures[0] == null_handle)
            {
//                log_error("shader not specified, assuming \"default_0\"");
                shader = GfxRenderer->Fetch_Shader("default_0");
            }
            else if (textures[1] == null_handle)
            {
//                log_error("shader not specified, assuming \"default_1\"");
                shader = GfxRenderer->Fetch_Shader("default_1");
            }
            else if (textures[2] == null_handle)
            {
//                log_error("shader not specified, assuming \"default_2\"");
                shader = GfxRenderer->Fetch_Shader("default_2");
            }
        }

        // TBD, TODO: move material validation to renderer, to eliminate branching?
        if( Global.GfxRenderer == "default" ) {

            if( !shader ) {
                is_good = false;
                return;
            }

            for (auto it : parse_info->param_mapping)
            {
                std::string key = it.first;
                glm::vec4 value = it.second.data;

                if (key.size() > 1 && key[0] != '_')
                {
                    size_t num = std::stoi(key) - 1;
                    if (num < gl::MAX_PARAMS) {
                        params[num] = value;
                    }
                    else {
                        log_error("invalid param binding: " + std::to_string(num));
                        is_good = false;
                    }
                }
                else if (key.size() > 2)
                {
                    key.erase(0, 1);
                    key.pop_back();
                    if (shader->param_conf.find(key) != shader->param_conf.end())
                    {
                        gfx::shader_param_entry entry = shader->param_conf[key];
                        for (size_t i = 0; i < entry.size; i++)
                            params[entry.location][entry.offset + i] = value[i];
                    }
                    else {
                        log_error("unknown param binding: " + key);
                        is_good = false;
                    }
                }
                else {
                    log_error("unrecognized param binding: " + key);
                    is_good = false;
                }
            }
        }
        parse_info.reset();
    }

    if( Global.GfxRenderer != "default" ) {
        // basic texture validation for legacy branch
        for( auto texturehandle : textures ) {
            if( texturehandle == null_handle ) {
                break;
            }
            if( GfxRenderer->Texture( texturehandle ).get_id() <= 0 ) {
                is_good = false;
            }
        }
        return;
    }

    if( !shader ) {
        is_good = false;
        return;
    }

    for (auto it : shader->param_conf)
    {
        gfx::shader_param_entry entry = it.second;
        if (std::isnan(params[entry.location][entry.offset]))
        {
            float value = std::numeric_limits<float>::quiet_NaN();
            if (entry.defaultparam == gfx::shader_param_default::one)
                value = 1.0f;
            else if (entry.defaultparam == gfx::shader_param_default::zero)
                value = 0.0f;
            else if (entry.defaultparam == gfx::shader_param_default::required)
                log_error("unspecified required param: " + it.first);
            else if (entry.defaultparam != gfx::shader_param_default::nan)
            {
                params_state.push_back(entry);
                continue;
            }

            for (size_t i = 0; i < entry.size; i++)
                params[entry.location][entry.offset + i] = value;
        }
    }

    for (auto it : shader->texture_conf)
    {
        gfx::shader_texture_entry &entry = it.second;
        texture_handle handle = textures[entry.id];
        // NOTE: texture validation at this stage relies on forced texture load behaviour during its create() call
        // TODO: move texture id validation to later stage if/when deferred texture loading is implemented
        if( handle
         && GfxRenderer->Texture(handle).get_id() > 0 ) {
            GfxRenderer->Texture(handle).set_components_hint((GLint)entry.components);
        }
        else {
            log_error("missing texture: " + it.first);
            is_good = false;
        }
    }
}

bool opengl_material::update() {

    auto const texturepathbackup { Global.asCurrentTexturePath };
    auto const namebackup { name };
    auto const pathbackup { path };
    cParser materialparser( name + ".mat", cParser::buffer_FILE ); // fairly safe to presume .mat is present for branching materials

    // temporarily set texture path to state recorded in the material
    Global.asCurrentTexturePath = paths.data[ path ];

    // clean material slate, restore relevant members
    *this = opengl_material();
    name = namebackup;
    path = pathbackup;

    auto result { false };

    if( true == deserialize( materialparser, true ) ) {
        try {
            finalize( true );
            result = true;
        }
        catch( gfx::shader_exception const &e ) {
            ErrorLog( "invalid shader: " + std::string( e.what() ) );
        }
    }
    // restore texture path
    Global.asCurrentTexturePath = texturepathbackup;

    return result;
}

std::unordered_set<std::string> seasons = {
    "winter:", "spring:", "summer:", "autumn:"
};

bool is_season( std::string const &String ) {

    return seasons.find(String) != seasons.end();
}

std::unordered_set<std::string> weather = {
    "clear:", "cloudy:", "rain:", "snow:" };

bool is_weather( std::string const &String ) {

    return weather.find(String) != weather.end();
}

// imports member data pair from the config file
void opengl_material::deserialize_texture_mapping( cParser &Input, std::string const &Key, int const Priority ) {

    // a texture binding can be provided as a pool of alternatives, "textureN_variants: [ a b c ]".
    // unlike a regular random set, which is resolved once when the material is loaded and thus shared
    // by everything using it, such pool is preserved whole and one entry is picked per model instance
    auto key { Key };
    auto const variantmarker { key.find( "_variants" ) };
    auto const isvariantset { variantmarker != std::string::npos };
    if( true == isvariantset ) {
        // reduce the key to its plain form, "N_variants:" becomes "N:"
        key.erase( variantmarker, std::string( "_variants" ).size() );
    }

    std::vector<std::string> variants;
    std::string value;
    if( false == isvariantset ) {
        value = deserialize_random_set( Input );
        replace_slashes( value );
    }
    else {
        variants = deserialize_set( Input );
        for( auto &variant : variants ) {
            replace_slashes( variant );
        }
        if( true == variants.empty() ) {
            log_error( "empty texture variant pool: " + key );
            return;
        }
        // first entry doubles as the regular, variant-agnostic texture of the material
        value = variants.front();
    }

    auto const [ it, inserted ] { parse_info->tex_mapping.try_emplace( key, parse_info_s::tex_def{ value, Priority, variants } ) };
    if( ( false == inserted ) && ( Priority > it->second.priority ) ) {
        it->second = parse_info_s::tex_def{ value, Priority, variants };
    }
}

bool
opengl_material::deserialize_mapping( cParser &Input, int const Priority, bool const Loadnow ) {

    // NOTE: comma can be part of legacy file names, so we don't treat it as a separator here
    auto key { Input.getToken<std::string>( true, "\n\r\t  ;[]" ) };
    // key can be an actual key or block end
    if( true == key.empty() || key == "}" ) { return false; }

    if( Priority != -1 ) {
        // regular attribute processing mode

        // mark potential material change
        update_on_weather_change |= is_weather( key );
        update_on_season_change  |= is_season( key );

        if( key == Global.Weather ) {
            // weather textures override generic (pri 0) and seasonal (pri 1) textures
            // seasonal weather textures (pri 1+2=3) override generic weather (pri 2) textures
            // skip the opening bracket
            auto const value { Input.getToken<std::string>( true, "\n\r\t ;" ) };
            while( true == deserialize_mapping( Input, Priority + 2, Loadnow ) ) {
                ; // all work is done in the header
            }
        }
        else if( key == Global.Season ) {
            // seasonal textures override generic textures
            // skip the opening bracket
            auto const value { Input.getToken<std::string>( true, "\n\r\t ;" ) };
            while( true == deserialize_mapping( Input, Priority + 1, Loadnow ) ) {
                ; // all work is done in the header
            }
        }

        else if (key.compare(0, 7, "texture") == 0) {
            key.erase(0, 7);
            deserialize_texture_mapping( Input, key, Priority );
        }
        else if (key.compare(0, 5, "param") == 0) {
            key.erase(0, 5);

            std::string value = Input.getToken<std::string>( true, "\n\r\t;" );
            std::istringstream stream(value);
            glm::vec4 data;
            stream >> data.r;
            stream >> data.g;
            stream >> data.b;
            stream >> data.a;

            auto it = parse_info->param_mapping.find(key);
            if (it == parse_info->param_mapping.end())
                parse_info->param_mapping.emplace(std::make_pair(key, parse_info_s::param_def({ data, Priority })));
            else if (Priority > it->second.priority)
            {
                parse_info->param_mapping.erase(it);
                parse_info->param_mapping.emplace(std::make_pair(key, parse_info_s::param_def({ data, Priority })));
            }
        }
        else if (key == "shader:" &&
                (!shader || Priority > m_shader_priority))
        {
            try
            {
                std::string value = deserialize_random_set( Input );
                shader = GfxRenderer->Fetch_Shader(value);
                m_shader_priority = Priority;
            }
            catch (gfx::shader_exception const &e)
            {
                log_error("invalid shader: " + std::string(e.what()));
            }
        }
        else if (key == "opacity:" &&
                Priority > m_opacity_priority)
        {
            std::string value = deserialize_random_set( Input );
            opacity = std::stof(value); //m7t: handle exception
            m_opacity_priority = Priority;
        }
        else if (key == "selfillum:" &&
                Priority > m_selfillum_priority)
        {
            std::string value = deserialize_random_set( Input );
            selfillum = std::stof(value); //m7t: handle exception
            m_selfillum_priority = Priority;
        }
        else if (key == "glossiness:" &&
                Priority > m_glossiness_priority)
        {
            std::string value = deserialize_random_set( Input );
            glossiness = std::stof(value); //m7t: handle exception
            m_glossiness_priority = Priority;
        }
        else if (key == "shadow_rank:")
        {
            auto const value { deserialize_random_set( Input ) };
            shadow_rank = std::stof(value); //m7t: handle exception
        }
        else if( key == "size:" ) {
            Input.getTokens( 2 );
            Input
                >> size.x
                >> size.y;
        }
        else {
            auto const value = Input.getToken<std::string>( true, "\n\r\t ;" );
            if( value == "{" ) {
                // unrecognized or ignored token, but comes with attribute block and potential further nesting
                // go through it and discard the content
                while( true == deserialize_mapping( Input, -1, Loadnow ) ) {
                    ; // all work is done in the header
                }
            }
        }
    }
    else {
        // discard mode; ignores all retrieved tokens
        auto const value { Input.getToken<std::string>( true, "\n\r\t ;" ) };
        if( value == "{" ) {
            // ignored tokens can come with their own blocks, ignore these recursively
            // go through it and discard the content
            while( true == deserialize_mapping( Input, -1, Loadnow ) ) {
                ; // all work is done in the header
            }
        }
    }

    return true; // return value marks a key: value pair was extracted, nothing about whether it's recognized
}

float opengl_material::get_or_guess_opacity() const {

    if( opacity ) {
        return opacity.value();
    }
    if (textures[0] != null_handle)
    {
        auto const &tex = GfxRenderer->Texture(textures[0]);
        if (tex.get_has_alpha())
            return 0.0f;
        else
            return 0.5f;
    }

    return 0.5f;
}

bool
opengl_material::is_translucent() const {

    return textures[0] != null_handle ? GfxRenderer->Texture(textures[0]).get_has_alpha() : false;
}

// create material object from data stored in specified file.
// NOTE: the deferred load parameter is passed to textures defined by material, the material itself is always loaded immediately
material_handle
material_manager::create( std::string const &Filename, bool const Loadnow ) {

    auto filename { Filename };

    if( contains( filename, '|' ) ) {
        filename.erase( filename.find( '|' ) ); // po | może być nazwa kolejnej tekstury
    }

    // discern references to textures generated by a script
    // TBD: support file: for file resources?
    auto const isgenerated { filename.find( "make:" ) == 0 || filename.find( "internal_src:" ) == 0 };

    // process supplied resource name
    if( isgenerated ) {
        // generated resource
        // scheme:(user@)path?query

        // remove scheme indicator
        filename.erase( 0, filename.find(':') + 1 );
        // TBD, TODO: allow shader specification as part of the query?
        erase_leading_slashes( filename );
    }
    else {
        // regular file resource
        // (filepath/)filename.extension

        erase_extension( filename );
        replace_slashes( filename );
        erase_leading_slashes( filename );
    }

    auto const databanklookup { find_in_databank( filename ) };
    if( databanklookup != null_handle ) {
        return databanklookup;
    }

    opengl_material material;
    material_handle materialhandle { null_handle };

    auto const locator {
        isgenerated ?
            std::make_pair( filename, "make:" ) :
            find_on_disk( filename ) };

    if( false == isgenerated
     && false == locator.first.empty() ) {
        // try to parse located file resource
        cParser materialparser(
            locator.first + locator.second,
            cParser::buffer_FILE );
        if( true == material.deserialize( materialparser, Loadnow ) ) {
            material.name = locator.first;
        }
    }
    else {
        // if there's no .mat file, this can be either autogenerated texture,
        // or legacy method of referring just to diffuse texture directly.
        // wrap basic material around it in either case
        auto const texturehandle { GfxRenderer->Fetch_Texture( Filename, Loadnow ) };
		if( texturehandle != null_handle ) {
			// use texture path and name to tell the newly created materials apart
			material.name = GfxRenderer->Texture( texturehandle ).get_name();
/*
            // material would attach default shader anyway, but it would spit to error log
	        try
	        {
	            material.shader = GfxRenderer->Fetch_Shader("default_1");
	        }
	        catch (gfx::shader_exception const &e)
	        {
	            ErrorLog("invalid shader: " + std::string(e.what()));
	        }
        }
*/
            // HACK: create parse info for material finalize() method
            cParser materialparser(
                "texture1: \"" + Filename + "\"",
                cParser::buffer_TEXT );
            material.deserialize( materialparser, Loadnow );
        }
    }

	if( false == material.name.empty() ) {
		// if we have material name and shader it means resource was processed succesfully.
		//
		// IMPORTANT: capture the handle ONLY after emplace_back succeeds. The
		// previous version pre-assigned `materialhandle = m_materials.size()`
		// and then ran `finalize(Loadnow)` (which can throw shader_exception
		// when a shader fails to compile) and `emplace_back` together inside
		// the try. If finalize() threw, emplace_back never ran, but the
		// handle was already set to size() -- pointing one past the actual
		// last element. Subsequent `m_materials[handle]` lookups would then
		// read garbage past end-of-vector, producing the classic 0x30-offset
		// access violation seen in TSubModel::BinInit (vtable read on a
		// bogus IMaterial pointer). Now we leave materialhandle as
		// null_handle if anything throws, and the caller treats it as a
		// failed-to-load material.
        try {
			material.finalize(Loadnow);
			materialhandle = m_materials.size();
			m_materials.emplace_back( std::move(material) );
			m_materialmappings.emplace( m_materials.back().name, materialhandle );
		} catch (gfx::shader_exception const &e) {
			ErrorLog("invalid shader: " + std::string(e.what()));
			// record the failure so subsequent Fetch_Material(filename) calls
			// short-circuit without re-running the failing compile.
			m_materialmappings.emplace( material.name, null_handle );
		}
    }
    else {
        // otherwise record our failure to process the resource, to speed up subsequent attempts
        m_materialmappings.emplace( filename, materialhandle );
    }

    return materialhandle;
};

material_handle
material_manager::create_from_text( std::string const &Name, std::string const &Definition ) {

    opengl_material material;
    cParser materialparser( Definition, cParser::buffer_TEXT );
    if( false == material.deserialize( materialparser, true ) ) {
        return null_handle;
    }
    material.name = Name;
    try {
        material.finalize( true );
    }
    catch( gfx::shader_exception const &e ) {
        ErrorLog( "invalid shader: " + std::string( e.what() ) );
        return null_handle;
    }
    auto const materialhandle { static_cast<material_handle>( m_materials.size() ) };
    m_materials.emplace_back( std::move( material ) );
    // not listed under its name: the name isn't a file which could be asked for
    return materialhandle;
}

namespace {

opengl_material
variant_of( opengl_material const &Source, std::string const &Suffix ) {

    opengl_material variant;
    variant.textures = Source.textures;
    variant.texture_variants = Source.texture_variants;
    variant.params = Source.params;
    variant.params_state = Source.params_state;
    variant.shader = Source.shader;
    variant.opacity = Source.opacity;
    variant.selfillum = Source.selfillum;
    variant.shadow_rank = Source.shadow_rank;
    variant.name = Source.name + Suffix;
    variant.size = Source.size;
    variant.is_good = Source.is_good;
    variant.path = Source.path;
    return variant;
}

} // namespace

material_handle
material_manager::create_matte( material_handle const Material ) {

    if( Material == null_handle || Material >= m_materials.size() ) {
        return Material;
    }
    auto const lookup { m_mattes.find( Material ) };
    if( lookup != m_mattes.end() ) {
        return lookup->second;
    }
    auto matte { variant_of( m_materials[ Material ], ":matte" ) };
    matte.glossiness = 0.1f;
    if( matte.shader ) {
        for( auto const &[ key, value ] : std::vector<std::pair<std::string, glm::vec4>>{ { "weathering", glm::vec4{ 1.f } } } ) {
            auto const entry { matte.shader->param_conf.find( key ) };
            if( entry == matte.shader->param_conf.end() ) {
                continue;
            }
            for( std::size_t i = 0; i < entry->second.size; ++i ) {
                matte.params[ entry->second.location ][ entry->second.offset + i ] = value[ static_cast<glm::length_t>( i ) ];
            }
            matte.params_state.erase(
                std::remove_if(
                    std::begin( matte.params_state ), std::end( matte.params_state ),
                    [&]( gfx::shader_param_entry const &State ) {
                        return State.location == entry->second.location && State.offset == entry->second.offset; } ),
                std::end( matte.params_state ) );
        }
    }
    auto const mattehandle { static_cast<material_handle>( m_materials.size() ) };
    m_materials.emplace_back( std::move( matte ) );
    m_mattes.emplace( Material, mattehandle );
    return mattehandle;
}

material_handle
material_manager::create_tinted( material_handle const Material, glm::vec3 const &Colour ) {

    if( Material == null_handle || Material >= m_materials.size() || !m_materials[ Material ].shader ) {
        return Material;
    }
    auto const key { std::make_tuple( Material, Colour.r, Colour.g, Colour.b ) };
    if( auto const lookup { m_tints.find( key ) }; lookup != m_tints.end() ) {
        return lookup->second;
    }
    auto const &conf { m_materials[ Material ].shader->param_conf };
    auto const weathering { conf.find( "weathering" ) };
    if( weathering == conf.end()
     || weathering->second.offset != 3
     || std::any_of(
            std::begin( conf ), std::end( conf ),
            [&]( auto const &Entry ) {
                return Entry.first != weathering->first && Entry.second.location == weathering->second.location; } ) ) {
        return Material;
    }
    auto tinted { variant_of( m_materials[ Material ], ":tinted" ) };
    tinted.glossiness = m_materials[ Material ].glossiness;
    auto const location { weathering->second.location };
    tinted.params[ location ] = glm::vec4{ Colour, -1.f };
    tinted.params_state.erase(
        std::remove_if(
            std::begin( tinted.params_state ), std::end( tinted.params_state ),
            [&]( gfx::shader_param_entry const &State ) {
                return State.location == location; } ),
        std::end( tinted.params_state ) );
    auto const tintedhandle { static_cast<material_handle>( m_materials.size() ) };
    m_materials.emplace_back( std::move( tinted ) );
    m_tints.emplace( key, tintedhandle );
    return tintedhandle;
}

namespace {

// the shaders a material is lit by which the blending shaders can light the same way, by the names they're fetched by, in the
// order of the lighting they stand for (1 normalmap, 2 sunless, 3 shadowless)
char const *const blend_normal_shaders[] = { "normalmap", "sunlessnormalmap", "shadowlessnormalmap" };
char const *const blend_specgloss_shaders[] = { "normalmap_specgloss", "sunlessnormalmap_specgloss", "shadowlessnormalmap_specgloss" };
int constexpr blend_specgloss { 4 }; // added to the lighting for a material with a specgloss map
int constexpr blend_parallax { 8 }; // added to the lighting (1, normalmap) of a parallax material

std::string
blend_shader( int const Lighting ) {

    std::string name { "trackblend" };
    if( Lighting & blend_specgloss ) { name += "_specgloss"; }
    else if( Lighting & blend_parallax ) { name += "_parallax"; }
    else if( Lighting > 0 ) { name += "_normalmap"; }
    return name;
}

} // namespace

int
material_manager::blend_lighting( material_handle const Material ) {

    if( Material == null_handle || Material >= m_materials.size() ) {
        return -1;
    }
    if( auto const lookup { m_blendables.find( Material ) }; lookup != m_blendables.end() ) {
        return lookup->second;
    }
    auto result { -1 };
    auto const &material { m_materials[ Material ] };
    auto const &conf { material.shader ? material.shader->texture_conf : decltype( material.shader->texture_conf ) {} };
    auto const has = [&]( char const *Name ) {
        auto const found { conf.find( Name ) };
        return found != conf.end() && material.textures[ found->second.id ] != null_handle; };
    if( material.is_good
     && material.shader
     && material.texture_variants.empty()
     // a tint takes the parameters the blending shader keeps its lighting in
     && false == ( material.params[ 2 ].w < 0.f )
     && has( "diffuse" ) ) {
        try {
            auto lighting { -1 };
            auto const lit_as = [&]( char const *const ( &Shaders )[ 3 ], int const Extra ) {
                for( int index = 0; index < 3; ++index ) {
                    if( material.shader == GfxRenderer->Fetch_Shader( Shaders[ index ] ) ) {
                        lighting = index + 1 + Extra;
                    }
                } };
            if( conf.size() == 1 ) {
                lighting = 0;
            }
            else if( conf.size() == 2 && has( "normalmap" ) ) {
                lit_as( blend_normal_shaders, 0 );
                if( material.shader == GfxRenderer->Fetch_Shader( "parallax" ) ) {
                    lighting = 1 + blend_parallax;
                }
            }
            else if( conf.size() == 3 && has( "normalmap" ) && has( "specgloss" ) ) {
                lit_as( blend_specgloss_shaders, blend_specgloss );
            }
            if( lighting >= 0 ) {
                // the parameters of the material go to the blending one as they are, so they have to be in the same places in both
                auto const blend { GfxRenderer->Fetch_Shader( blend_shader( lighting ) ) };
                auto const fits { std::all_of(
                    std::begin( material.shader->param_conf ), std::end( material.shader->param_conf ),
                    [&]( auto const &Entry ) {
                        auto const found { blend->param_conf.find( Entry.first ) };
                        return found != blend->param_conf.end()
                            && found->second.location == Entry.second.location
                            && found->second.offset == Entry.second.offset
                            && found->second.size == Entry.second.size; } ) };
                result = ( fits ? lighting : -1 );
            }
        }
        catch( gfx::shader_exception const &e ) {
            ErrorLog( "invalid shader: " + std::string( e.what() ) );
        }
    }
    if( result < 0 ) {
        // once for each material a track would blend, which of the conditions it fails
        std::string textures;
        for( auto const &entry : conf ) {
            textures += ( textures.empty() ? "" : ", " ) + entry.first;
        }
        WriteLog(
            "track blend: material \"" + material.name + "\" isn't blended:"
            + ( material.is_good ? "" : " not good;" )
            + ( material.shader ? " shader \"" + GfxRenderer->Shader_Name( material.shader ) + "\" with the textures " + textures + ";" : " no shader;" )
            + ( material.texture_variants.empty() ? "" : " texture variants;" )
            + ( material.params[ 2 ].w < 0.f ? " tinted;" : "" ) );
    }
    m_blendables.emplace( Material, result );
    return result;
}

bool
material_manager::blendable( material_handle const Material ) {

    return blend_lighting( Material ) >= 0;
}

bool
material_manager::build_blend( blend_key const &Key, opengl_material &Blend ) {

    auto const material { std::get<0>( Key ) };
    std::array<material_handle, 3> const others { std::get<1>( Key ), std::get<2>( Key ), std::get<3>( Key ) };
    auto const lighting { blend_lighting( material ) };
    if( lighting < 0 ) {
        return false;
    }
    auto const texture_of = [&]( material_handle const Source, char const *Name ) -> texture_handle {
        if( false == blendable( Source ) ) {
            return null_handle;
        }
        auto const &source { m_materials[ Source ] };
        auto const found { source.shader->texture_conf.find( Name ) };
        return ( found != source.shader->texture_conf.end() ? source.textures[ found->second.id ] : null_handle );
    };
    // the slots of the textures in the blending shaders (mat_trackblend*.frag), parallax as normalmap: with a specgloss map there's room for the textures
    // of two ends alone, p1 and p2, which is all the rails of a plain track have
    auto const normals { lighting > 0 };
    auto const specgloss { ( lighting & blend_specgloss ) != 0 };
    int const diffuseslots[] = { 0, 1, 2, 3 };
    int const normalslots[] = { ( specgloss ? 3 : 4 ), ( specgloss ? 4 : 5 ), ( specgloss ? 5 : 6 ), 7 };
    auto const ends { specgloss ? 2 : 3 };
    try {
        Blend = variant_of( m_materials[ material ], ":blend" );
        Blend.shader = GfxRenderer->Fetch_Shader( blend_shader( lighting ) );
        Blend.texture_variants.clear();
        Blend.textures.fill( null_handle );
        auto const owndiffuse { texture_of( material, "diffuse" ) };
        auto const ownnormal { texture_of( material, "normalmap" ) };
        Blend.textures[ diffuseslots[ 0 ] ] = owndiffuse;
        if( normals ) {
            Blend.textures[ normalslots[ 0 ] ] = ownnormal;
        }
        if( specgloss ) {
            Blend.textures[ 6 ] = texture_of( material, "specgloss" );
        }
        for( int end = 0; end < ends; ++end ) {
            // the slots of the ends nothing is blended at take the own textures, the shader samples them only with a weight; a
            // material without a normal map blended into one with it takes its normal map
            auto const diffuse { texture_of( others[ end ], "diffuse" ) };
            Blend.textures[ diffuseslots[ 1 + end ] ] = ( diffuse != null_handle ? diffuse : owndiffuse );
            if( normals ) {
                auto const normal { texture_of( others[ end ], "normalmap" ) };
                Blend.textures[ normalslots[ 1 + end ] ] = ( normal != null_handle ? normal : ownnormal );
            }
        }
        // lit as the shader of the material: 0 mat_default, 1 normalmap, 2 sunless, 3 shadowless
        Blend.params[ 2 ].x = static_cast<float>( lighting & 3 );
        // the parameters taken from the bindings again, by the shader
        Blend.params_state.clear();
        Blend.finalize( true );
        return Blend.is_good;
    }
    catch( gfx::shader_exception const &e ) {
        ErrorLog( "invalid shader: " + std::string( e.what() ) );
    }
    return false;
}

material_handle
material_manager::create_blend( material_handle const Material, std::array<material_handle, 3> const &Others ) {

    if( false == blendable( Material ) ) {
        return Material;
    }
    blend_key const key { Material, Others[ 0 ], Others[ 1 ], Others[ 2 ] };
    if( auto const lookup { m_blends.find( key ) }; lookup != m_blends.end() ) {
        return lookup->second;
    }
    auto result { Material };
    opengl_material blend;
    if( build_blend( key, blend ) ) {
        result = static_cast<material_handle>( m_materials.size() );
        m_materials.emplace_back( std::move( blend ) );
        m_blendsources.emplace_back( result, key );
    }
    m_blends.emplace( key, result );
    return result;
}

void
material_manager::rebuild_blends() {

    if( m_blendsources.empty() ) {
        return;
    }
    // the materials blended took the textures of another season or weather, the blending ones take them anew in their places
    m_blendables.clear();
    for( auto const &[ handle, key ] : m_blendsources ) {
        opengl_material blend;
        if( build_blend( key, blend ) ) {
            m_materials[ handle ] = std::move( blend );
        }
        else {
            // can't be blended any more: drawn as the material of the track itself, the weights of its vertices unused
            m_materials[ handle ] = variant_of( m_materials[ std::get<0>( key ) ], ":blend" );
        }
    }
}

void
material_manager::on_weather_change() {

    for( auto &material : m_materials ) {
        if( material.update_on_weather_change ) { material.update(); }
    }
    rebuild_blends();
}

void
material_manager::on_season_change() {

    for( auto &material : m_materials ) {
        if( material.update_on_season_change ) { material.update(); }
    }
    rebuild_blends();
}

// checks whether specified material is in the material bank. returns handle to the material, or a null handle
material_handle
material_manager::find_in_databank( std::string const &Materialname ) const {

    std::vector<std::string> const filenames {
        Global.asCurrentTexturePath + Materialname,
        Materialname,
        paths::textures + Materialname };

    for( auto const &filename : filenames ) {
        auto const lookup { m_materialmappings.find( filename ) };
        if( lookup != m_materialmappings.end() ) {
            return lookup->second;
        }
    }
    // all lookups failed
    return null_handle;
}

// checks whether specified file exists.
// NOTE: technically could be static, but we might want to switch from global texture path to instance-specific at some point
std::pair<std::string, std::string>
material_manager::find_on_disk( std::string const &Materialname ) {

    auto const materialname { ToLower( Materialname ) };
    return FileExists({Global.asCurrentTexturePath + materialname, materialname, paths::textures + materialname}, {".mat"});
}

//---------------------------------------------------------------------------
