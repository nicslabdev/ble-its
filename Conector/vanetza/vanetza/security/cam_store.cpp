#include <vanetza/security/cam_store.hpp>

namespace vanetza
{
namespace security
{

uint16_t CamStore::insert(const Frame& frame)
{
    uint16_t id = index;
    printf("Stored frames ids: ");
    show_ids();
    m_frames.insert(std::make_pair(id, frame));
    index_vector.insert(index_vector.end(), id);
    index++;
    return id;
}

void CamStore::erase_id(uint16_t id){
    std::vector<uint16_t>::iterator it  = std::find(index_vector.begin(), index_vector.end(), id);
    index_vector.erase(it);
    printf("Stored frames ids: ");
    show_ids();
}

void CamStore::show_ids(){
    for(uint16_t i:index_vector){
        printf("%d ", i);
    }
    printf("\n");
}

std::vector<uint16_t> CamStore::obtain_ids(){
    return index_vector;
}

std::map<uint16_t, Frame>::iterator CamStore::lookup(uint16_t id)
{
    return m_frames.find(id);   
}

void CamStore::erase(uint16_t key){
    m_frames.erase(key);
}

std::map<uint16_t, Frame>::iterator CamStore::begin()
{
    return m_frames.begin();
}

std::map<uint16_t, Frame>::iterator CamStore::end()
{
    return m_frames.end();
}

size_t CamStore::size(){
    return m_frames.size();
}

int CamStore::at(uint16_t key, Frame f){
    int err;
    try{
        f = m_frames.at(key);
        err = 0;
    }catch(std::out_of_range e){
        printf("Error! Out of range\n");
        err = -1;
    }
    return err;
}

} // namespace security
} // namespace vanetza
