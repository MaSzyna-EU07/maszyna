/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
    MaSzyna EU07 locomotive simulator
    Copyright (C) 2001-2004  Marcin Wozniak and others

*/

#pragma once

#include "utilities/Classes.h"
#include "utilities/Float3d.h"
#include "model/Model3d.h"
#include "vehicle/DynObj.h"
#include "scene/scenenode.h"

const int iMaxNumLights = 8;
float const DefaultDarkThresholdLevel { 0.325f };

// typy stanu świateł
enum TLightState
{
	ls_Off = 0, // zgaszone
	ls_On = 1, // zapalone
	ls_Blink = 2, // migające
	ls_Dark = 3, // Ra: zapalajce się automatycznie, gdy zrobi się ciemno
	ls_Home = 4, // like ls_dark but off late at night
	ls_winter = 5 // turned on when its winter
};

class TAnimVocaloidFrame
{ // ramka animacji typu Vocaloid Motion Data z programu MikuMikuDance
  public:
    char cBone[15]; // nazwa kości, może być po japońsku
    int iFrame; // numer ramki
    float3 f3Vector; // przemieszczenie
    float4 qAngle; // kwaternion obrotu
    char cBezier[64]; // krzywe Béziera do interpolacji dla x,y,z i obrotu
};

class basic_event;

class TAnimContainer : std::enable_shared_from_this<TAnimContainer>
{ // opakowanie submodelu, określające animację egzemplarza - obsługiwane jako lista
    friend TAnimModel;

  private:
	glm::vec3 vRotateAngles; // dla obrotów Eulera
	glm::vec3 vDesiredAngles;
    double fRotateSpeed;
	glm::dvec3 vTranslation;
	glm::dvec3 vTranslateTo;
    double fTranslateSpeed; // może tu dać wektor?
    float4 qCurrent; // aktualny interpolowany
    float4 qStart; // pozycja początkowa (0 dla interpolacji)
    float4 qDesired; // pozycja końcowa (1 dla interpolacji)
    float fAngleCurrent; // parametr interpolacyjny: 0=start, 1=docelowy
    float fAngleSpeed; // zmiana parametru interpolacji w sekundach
  public:
    TSubModel *pSubModel;

  private:
	std::shared_ptr<float4x4> mAnim; // macierz do animacji kwaternionowych
    // dla kinematyki odwróconej używane są kwaterniony
    float fLength; // długość kości dla IK
    int iAnim; // animacja: +1-obrót Eulera, +2-przesuw, +4-obrót kwaternionem, +8-IK
    //+0x80000000: animacja z eventem, wykonywana poza wyświetlaniem
    //+0x100: pierwszy stopień IK - obrócić w stronę pierwszego potomnego (dziecka)
    //+0x200: drugi stopień IK - dostosować do pozycji potomnego potomnego (wnuka)
    basic_event *evDone; // ewent wykonywany po zakończeniu animacji, np. zapór, obrotnicy
  public:
    // wyświetlania
	TAnimContainer();
    bool Init(TSubModel *pNewSubModel);
    inline
    std::string NameGet() {
        return (pSubModel ? pSubModel->pName : ""); };
    void SetRotateAnim( glm::vec3 vNewRotateAngles, double fNewRotateSpeed);
    void SetTranslateAnim( glm::dvec3 vNewTranslate, double fNewSpeed);
    void AnimSetVMD(double fNewSpeed);
    void PrepareModel();
    void UpdateModel();
    void UpdateModelIK();
    bool InMovement(); // czy w trakcie animacji?
    inline
    double AngleGet() {
        return vRotateAngles.z; }; // jednak ostatnia, T3D ma inny układ
    inline
    glm::dvec3 TransGet() {
        return glm::dvec3(-vTranslation.x, vTranslation.z, vTranslation.y); }; // zmiana, bo T3D ma inny układ
    inline
    void WillBeAnimated() {
        if (pSubModel)
            pSubModel->WillBeAnimated(); };
    void EventAssign(basic_event *ev);
    inline
    basic_event * Event() {
        return evDone; };
};

// opakowanie modelu, określające stan egzemplarza
class TAnimModel : public scene::basic_node {

    friend opengl_renderer;
    friend opengl33_renderer;
    friend itemproperties_panel;

public:
// constructors
    explicit TAnimModel( scene::node_data const &Nodedata );
// methods
    static void AnimUpdate( double dt );
    bool Init(std::string const &asName, std::string const &asReplacableTexture);
    bool Load(cParser *parser, bool ter = false);
	std::shared_ptr<TAnimContainer> AddContainer(std::string const &Name);
	std::shared_ptr<TAnimContainer> GetContainer(std::string const &Name = "");
	void LightSet( int const n, float const v );
    void SkinSet( int const Index, material_handle const Material );
	std::optional<std::tuple<float, float, std::optional<glm::vec3> > > LightGet( int const n );
    int TerrainCount();
    TSubModel * TerrainSquare(int n);
    int Flags();
	void on_season_update();
    inline
    material_data const *
        Material() const {
            return &m_materialdata; }
    inline
    TModel3d *
        Model() const {
            return pModel; }
    inline
    void
        Angles( glm::vec3 const &Angles ) {
            vAngle = Angles; }
    inline
    glm::vec3
        Angles() const {
            return vAngle; }
    // per-axis scale, applied between rotation and the submodel-local transform chain.
    // (1,1,1) = unchanged. Set by the `scale`/`endscale` scenario directives or the
    // optional `scale <factor>` / `scale <x> <y> <z>` token inside a `node model` block.
    // Per-axis values let you stretch a model along a single dimension; uniform input
    // (single float) broadcasts to all three axes.
    inline
    void
        Scale( glm::vec3 const &Factor ) {
            m_scale = glm::vec3(
                Factor.x > 0.0f ? Factor.x : 1.0f,
                Factor.y > 0.0f ? Factor.y : 1.0f,
                Factor.z > 0.0f ? Factor.z : 1.0f ); }
    inline
    void
        Scale( float const Factor ) {
            Scale( glm::vec3( Factor ) ); }
    inline
    glm::vec3 const &
        Scale() const {
            return m_scale; }
// members
	std::list<std::shared_ptr<TAnimContainer>> m_animlist;

	// lista animacji z eventem, które muszą być przeliczane również bez wyświetlania
	static std::list<std::weak_ptr<TAnimContainer>> acAnimList;

public:
// methods
    void RaPrepare(); // ustawienie animacji egzemplarza na wzorcu
    void RaAnimate( unsigned int const Framestamp ); // przeliczenie animacji egzemplarza

    // radius() subclass details, calculates node's bounding radius
    float radius_();
    // serialize() subclass details, sends content of the subclass to provided stream
    void serialize_( std::ostream &Output ) const;
    // deserialize() subclass details, restores content of the subclass from provided stream
    void deserialize_( std::istream &Input );
    // export() subclass details, sends basic content of the class in legacy (text) format to provided stream
    void export_as_text_( std::ostream &Output ) const;
    // checks whether provided token is a legacy (text) format keyword
    bool is_keyword( std::string const &Token ) const;

// members
	std::shared_ptr<TAnimContainer> pRoot; // pojemniki sterujące, tylko dla aniomowanych submodeli
    TModel3d *pModel { nullptr };
    glm::vec3 vAngle; // bazowe obroty egzemplarza względem osi
    glm::vec3 m_scale { 1.0f, 1.0f, 1.0f }; // per-axis scale (see Scale() accessors above)
    material_data m_materialdata;

    std::string asText; // tekst dla wyświetlacza znakowego
    // TODO: wrap into a light state struct, remove fixed element count
    int iNumLights { 0 };
    std::array<TSubModel *, iMaxNumLights> LightsOn {}; // Ra: te wskaźniki powinny być w ramach TModel3d
    std::array<TSubModel *, iMaxNumLights> LightsOff {};
	TSubModel *sm_winter_variant {}; // submodel zimowego wariantu
	TSubModel *sm_spring_variant {}; // submodel wiosennego wariantu
	TSubModel *sm_summer_variant {}; // submodel letniego wariantu
	TSubModel *sm_autumn_variant {}; // submodel jesiennego wariantu
    std::array<float, iMaxNumLights> lsLights {}; // ls_Off
    std::array<glm::vec3, iMaxNumLights> m_lightcolors; // -1 in constructor
    std::array<float, iMaxNumLights> m_lighttimers {};
    std::array<float, iMaxNumLights> m_lightopacities; // {1} in constructor
    float fOnTime { 1.f / 2 };// { 60.f / 45.f / 2 };
    float fOffTime { 1.f / 2 };// { 60.f / 45.f / 2 }; // były stałymi, teraz mogą być zmienne dla każdego egzemplarza
//    float fTransitionTime { fOnTime * 0.9f }; // time
    bool m_transition { true }; // smooth transition between light states
    unsigned int m_framestamp { 0 }; // id of last rendered gfx frame

    // true if this instance is eligible for the batched/instanced render path:
    // no lights, no submodel animations, no replacable skins, no seasonal variants.
    // computed once at end of Load() so the renderer can simply test the flag.
    bool m_instanceable { false };
    // helper: evaluates current state and updates m_instanceable accordingly.
    void update_instanceable_flag();

    // diagnostic counters (process-wide). Updated inside update_instanceable_flag()
    // so the renderer can surface load-time classification stats in the debug overlay.
    static int s_instanceable_total;       // count of TAnimModel*'s classified instanceable
    static int s_classified_total;         // count of TAnimModel*'s that ran through the classifier
    static int s_rejected_no_pmodel;
    static int s_rejected_lights;
    static int s_rejected_animlist;
    static int s_rejected_animated_submodel;
};



class instance_table : public basic_table<TAnimModel> {

};

//---------------------------------------------------------------------------
