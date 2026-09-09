


//////////////////////////////////////////////////////////////////////////////////////////
// cStars -- simple starfield model, simulating appearance of starry sky
module;

export module eu07.environment.stars;
import eu07.simcore;
import eu07.utilities.classes;

export {

class cStars {
public:
// types:

// methods:
    void init();
// constructors:
    cStars() = default;
// deconstructor:

// members:

public:
    // read directly by the renderers. They used to be friends, but a friend
    // declaration naming a class from another module would require importing it,
    // and that import closes a cycle the module graph does not allow.
// members:
    float m_longitude{ 19.0f }; // geograpic coordinates hardcoded roughly to Poland location, for the time being
    float m_latitude{ 52.0f };
    TModel3d *m_stars { nullptr };
};

}  // export
