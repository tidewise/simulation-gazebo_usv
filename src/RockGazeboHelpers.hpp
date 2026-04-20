#ifndef GAZEBO_USV_ROCK_GAZEBO_HELPER_HPP
#define GAZEBO_USV_ROCK_GAZEBO_HELPER_HPP

#include <Eigen/Geometry>
#include <base/Float.hpp>
#include <base/samples/Frame.hpp>
#include <gz/math.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/details/image.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/vector2d.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

namespace rock_gazebo_helpers {
    class SensorTask;

    /** Get a typed parameter from the given element
     *
     * If the parameter does not exist, use a default value
     */
    template <class T>
    T getParameter(std::string plugin_name,
        sdf::ElementConstPtr element,
        std::string parameter_name,
        std::string dimension,
        T default_value)
    {
        if (element->HasElement(parameter_name.c_str())) {
            T var = element->Get<T>(parameter_name.c_str());
            gzmsg << plugin_name << ": " << parameter_name << "=" << var << " "
                  << dimension << std::endl;
            return var;
        }
        else {
            gzmsg << plugin_name << ": " << parameter_name << " using default "
                  << default_value << " " << dimension << std::endl;
            return default_value;
        }
    }

    /** Get the scope of a topic that is relative to the given model
     */
    std::string computeModelTopicScope(gz::sim::Entity model,
        gz::sim::EntityComponentManager& ecm);

    /** Get the scope of a topic that is published by the given plugin
     */
    std::string computeModelTopicScope(gz::sim::Entity model,
        sdf::ElementConstPtr plugin,
        gz::sim::EntityComponentManager& ecm);

    std::string computePluginTopicScope(gz::sim::Entity model,
        sdf::ElementConstPtr plugin,
        gz::sim::EntityComponentManager& ecm);

    /** Resolve a link from name if an attribute provides it, or return the first link
     * of the model
     *
     * @param model the reference gazebo model. If no link is defined in the plugin
     *    description, its first link is returned instead.
     * @param plugin the plugin SDF definition
     * @param element_name the name of the element that contains the link name. If
     *    it not present, the function will return the first link of the model.
     */
    gz::sim::Entity resolveLinkWithDefault(gz::sim::Entity model,
        sdf::ElementConstPtr plugin_sdf,
        std::string const& element_name,
        gz::sim::EntityComponentManager& ecm);

    inline gz::math::Vector3d proto2Gz(gz::msgs::Vector3d const& gz)
    {
        return gz::math::Vector3d(gz.x(), gz.x(), gz.x());
    }

    inline gz::math::Vector2d proto2Gz(gz::msgs::Vector2d const& gz)
    {
        return gz::math::Vector2d(gz.x(), gz.x());
    }

    inline Eigen::Vector3d gz2Eigen(gz::math::Vector3d const& gz)
    {
        return Eigen::Vector3d(gz.X(), gz.Y(), gz.Z());
    }

    inline Eigen::Vector3d gz2Eigen(std::optional<gz::math::Vector3d> const& gz)
    {
        if (gz.has_value()) {
            return gz2Eigen(gz.value());
        }

        return Eigen::Vector3d(base::unknown<double>(),
            base::unknown<double>(),
            base::unknown<double>());
    }

    inline Eigen::Quaterniond gz2Eigen(gz::math::Quaterniond const& gz)
    {
        return Eigen::Quaterniond(gz.W(), gz.X(), gz.Y(), gz.Z());
    }

    inline Eigen::Quaterniond gz2Eigen(std::optional<gz::math::Quaterniond> const& gz)
    {
        if (gz.has_value()) {
            return gz2Eigen(gz.value());
        }

        return Eigen::Quaterniond(base::unknown<double>(),
            base::unknown<double>(),
            base::unknown<double>(),
            base::unknown<double>());
    }

    inline Eigen::Isometry3d gz2Eigen(gz::math::Pose3d const& gz)
    {
        Eigen::Vector3d pos = gz2Eigen(gz.Pos());
        Eigen::Quaterniond rot = gz2Eigen(gz.Rot());
        Eigen::Isometry3d pose;
        pose.setIdentity();
        pose.translate(pos);
        pose.rotate(rot);
        return pose;
    }

    inline Eigen::Isometry3d gz2Eigen(std::optional<gz::math::Pose3d> const& gz)
    {
        if (gz.has_value()) {
            return gz2Eigen(gz.value());
        }

        Eigen::Isometry3d pose;
        pose.matrix() *= base::unknown<double>();
        return pose;
    }

    inline gz::math::Vector3d eigen2Gz(Eigen::Vector3d const& gz)
    {
        return gz::math::Vector3d(gz.x(), gz.y(), gz.z());
    }

    inline std::pair<int, base::samples::frame::frame_mode_t> gzToRock(
        gz::msgs::PixelFormatType gz)
    {
        using namespace base::samples::frame;
        using std::make_pair;

        switch (gz) {
            case gz::msgs::RGB_INT8:
                return make_pair(8, MODE_RGB);
            case gz::msgs::RGB_INT16:
                return make_pair(16, MODE_RGB);
            case gz::msgs::RGB_INT32:
                return make_pair(32, MODE_RGB);
            case gz::msgs::RGBA_INT8:
                return make_pair(8, MODE_RGB32);
            case gz::msgs::BGR_INT8:
                return make_pair(8, MODE_BGR);
            case gz::msgs::BGR_INT16:
                return make_pair(16, MODE_BGR);
            case gz::msgs::BGR_INT32:
                return make_pair(32, MODE_BGR);
            case gz::msgs::BAYER_RGGB8:
                return make_pair(8, MODE_BAYER_RGGB);
            case gz::msgs::BAYER_BGGR8:
                return make_pair(8, MODE_BAYER_BGGR);
            case gz::msgs::BAYER_GBRG8:
                return make_pair(8, MODE_BAYER_GBRG);
            case gz::msgs::BAYER_GRBG8:
                return make_pair(8, MODE_BAYER_GRBG);
            default:
                throw std::invalid_argument(
                    "received image that cannot represented in Rock");
        }
    }

    inline std::list<std::string> splitScopedName(std::string const& scopedName)
    {
        std::list<std::string> result;
        std::string::size_type delim = scopedName.find("::"), current = 0;
        while (delim != std::string::npos) {
            result.push_back(scopedName.substr(current, delim));
            current = delim + 2;
            delim = scopedName.find("::", current);
        }
        result.push_back(scopedName.substr(current));
        return result;
    }

    inline gz::sim::Entity resolveSubmodelRecursive(gz::sim::Entity const& root,
        std::list<std::string> const& names,
        gz::sim::EntityComponentManager& ecm)
    {
        auto context = root;
        for (auto const& n : names) {
            auto child = gz::sim::Model(context).ModelByName(ecm, n);
            if (child == gz::sim::kNullEntity) {
                throw std::invalid_argument(
                    "could not find child model " + n + " of " +
                    gz::sim::scopedName(context, ecm, "::", false));
            }

            context = child;
        }

        return context;
    }

    inline gz::sim::Entity resolveSubmodelRecursive(gz::sim::Entity const& root,
        std::string const& scoped_name,
        gz::sim::EntityComponentManager& ecm)
    {
        return resolveSubmodelRecursive(root, splitScopedName(scoped_name), ecm);
    }

    inline gz::sim::Entity resolveJointRecursive(gz::sim::Entity const& root,
        std::string const& scopedName,
        gz::sim::EntityComponentManager& ecm)
    {
        auto names = splitScopedName(scopedName);

        auto jointName = names.back();
        names.pop_back();

        if (gz::sim::Model(root).Name(ecm) == names.front()) {
            names.pop_front();
        }

        auto submodel = resolveSubmodelRecursive(root, names, ecm);

        auto joint = gz::sim::Model(submodel).JointByName(ecm, jointName);
        if (joint == gz::sim::kNullEntity) {
            throw std::invalid_argument("could not find child joint " + jointName +
                                        " of " +
                                        gz::sim::scopedName(submodel, ecm, "::", false));
        }

        return joint;
    }

    inline gz::sim::Entity resolveLinkRecursive(gz::sim::Entity const& root,
        std::string const& scopedName,
        gz::sim::EntityComponentManager& ecm)
    {
        auto names = splitScopedName(scopedName);

        auto linkName = names.back();
        names.pop_back();

        auto submodel = resolveSubmodelRecursive(root, names, ecm);

        auto link = gz::sim::Model(submodel).LinkByName(ecm, linkName);
        if (link == gz::sim::kNullEntity) {
            throw std::invalid_argument("could not find child link " + linkName + " of " +
                                        gz::sim::scopedName(submodel, ecm, "::", false));
        }

        return link;
    }
}

#endif
