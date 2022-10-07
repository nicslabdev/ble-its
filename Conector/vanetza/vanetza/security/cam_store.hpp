#include <vanetza/security/certificate.hpp>
#include <list>
#include <map>


namespace vanetza
{
namespace security
{

typedef struct Frame{
    ByteBuffer sign_bytes; //bytes listos para realizar la firma
    HashedId8 hash; //hash del certificado
    boost::optional<vanetza::security::EcdsaSignature> sign_params; //firma

    //setters

    void set_sign_bytes(ByteBuffer b){
        sign_bytes = b;
    }

    void set_hash(HashedId8 h){
        hash = h;
    }

    void set_sign_params(boost::optional<vanetza::security::EcdsaSignature> s){
        sign_params = s;
    }

}Frame;

class CamStore
{
public:

    CamStore() = default;

    std::map<uint16_t, Frame>::iterator lookup(uint16_t id); //búsqueda de una trama dado su identificador

    uint16_t insert(const Frame& frame); //inserta la trama

    std::map<uint16_t, Frame>::iterator begin(); //devuelve un iterador al inicio del mapa

    std::map<uint16_t, Frame>::iterator end(); //devuelve un interador al final del mapa

    size_t size(); //número de tramas almacenadas

    int at(uint16_t key, Frame f);  //devuelve la trama dado su identificador

    void erase(uint16_t key); //elimina una trama

    void erase_id(uint16_t id); //elimina un identificador

    std::vector<uint16_t> obtain_ids(); //devuelve un vector con los identificadores de trama almacenados

    void show_ids(); //muestra los identificadores de las tramas almacenadas

private:

    std::map<uint16_t, Frame> m_frames; //mapa
    std::vector<uint16_t> index_vector; //vector con los id de las tramas por verificar
    uint16_t index = 0; //identificador que se asigna a la siguiente trama almacenada
};

} // namespace security
} // namespace vanetza

