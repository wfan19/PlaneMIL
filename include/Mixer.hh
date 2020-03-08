#ifndef MIXER_HH
#define MIXER_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "msgTypedefs.hh"
namespace gazebo
{
    class GZ_PLUGIN_VISIBLE Mixer : public ModelPlugin
    {
        public:
            Mixer();
            ~Mixer();

            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        private:

    };
}