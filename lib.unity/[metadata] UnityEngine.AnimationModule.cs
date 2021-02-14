using Unity;

namespace UnityEngine.AnimationModule {
    /// <summary>
    /// The mapping between a bone in the model and the conceptual bone in the Mecanim human anatomy.
    /// </summary>
    public struct HumanBone {
        /// <summary>
        /// The name of the bone to which the Mecanim human bone is mapped.
        /// </summary>
        public string boneName;
        /// <summary>
        /// The name of the Mecanim human bone to which the bone from the model is mapped.
        /// </summary>
        public string humanName;
        /// <summary>
        /// The rotation limits that define the muscle for this bone.
        /// </summary>
        public HumanLimit limit;
    }

    /// <summary>
    /// This class stores the rotation limits that define the muscle for a single human bone.
    /// </summary>
    public struct HumanLimit {
        /// <summary>
        /// Length of the bone to which the limit is applied.
        /// </summary>
        public float axisLength;
        /// <summary>
        /// The default orientation of a bone when no muscle action is applied.
        /// </summary>
        public Vector3 center;
        /// <summary>
        /// The maximum rotation away from the initial value that this muscle can apply.
        /// </summary>
        public Vector3 max;
        /// <summary>
        /// The maximum negative rotation away from the initial value that this muscle can apply.
        /// </summary>
        public Vector3 min;
        /// <summary>
        /// Should this limit use the default values?
        /// </summary>
        public bool useDefaultValues;
    }

    /// <summary>
    /// Class that holds humanoid avatar parameters to pass to the <c>AvatarBuilder.BuildHumanAvatar</c> function.
    /// </summary>
    public struct HumanDescription {
        /// <summary>
        /// Amount by which the arm's length is allowed to stretch when using IK.
        /// </summary>
        public float armStretch;
        /// <summary>
        /// Modification to the minimum distance between the feet of a humanoid model.
        /// </summary>
        public float feetSpacing;
        /// <summary>
        /// True for any human that has a translation Degree of Freedom (DoF). It is set to false by default.
        /// </summary>
        public bool hasTranslationDoF;
        /// <summary>
        /// Mapping between Mecanim bone names and bone names in the rig.
        /// </summary>
        public HumanBone[] human;
        /// <summary>
        /// Amount by which the leg's length is allowed to stretch when using IK.
        /// </summary>
        public float legStretch;
        /// <summary>
        /// Defines how the lower arm's roll/twisting is distributed between the elbow and wrist joints.
        /// </summary>
        public float lowerArmTwist;
        /// <summary>
        /// Defines how the lower leg's roll/twisting is distributed between the knee and ankle.
        /// </summary>
        public float lowerLegTwist;
        /// <summary>
        /// List of bone Transforms to include in the model.
        /// </summary>
        public SkeletonBone[] skeleton;
        /// <summary>
        /// Defines how the upper arm's roll/twisting is distributed between the shoulder and elbow joints.
        /// </summary>
        public float upperArmTwist;
        /// <summary>
        /// Defines how the upper leg's roll/twisting is distributed between the thigh and knee joints.
        /// </summary>
        public float upperLegTwist;
    }

    /// <summary>
    /// Details of the Transform name mapped to the skeleton bone of a model and its default position and rotation in the T-pose.
    /// </summary>
    public struct SkeletonBone {
        /// <summary>
        /// The name of the Transform mapped to the bone.
        /// </summary>
        public string name;
        /// <summary>
        /// The T-pose position of the bone in local space.
        /// </summary>
        public Vector3 position;
        /// <summary>
        /// The T-pose rotation of the bone in local space.
        /// </summary>
        public Quaternion rotation;
        /// <summary>
        /// The T-pose scaling of the bone in local space.
        /// </summary>
        public Vector3 scale;
    }

    /// <summary>
    /// Interface to control the Mecanim animation system.
    /// </summary>
    public class Animator : Behaviour {
        /// <summary>
        /// Gets the avatar angular velocity for the last evaluated frame.
        /// </summary>
        public Vector3 angularVelocity;
        /// <summary>
        /// Should root motion be applied?
        /// </summary>
        public bool applyRootMotion;
        /// <summary>
        /// Gets/Sets the current Avatar.
        /// </summary>
        public Avatar avatar;
        /// <summary>
        /// The position of the body center of mass.
        /// </summary>
        public Vector3 bodyPosition;
        /// <summary>
        /// The rotation of the body center of mass.
        /// </summary>
        public Quaternion bodyRotation;
        /// <summary>
        /// Controls culling of this Animator component.
        /// </summary>
        public AnimatorCullingMode cullingMode;
        /// <summary>
        /// Gets the avatar delta position for the last evaluated frame.
        /// </summary>
        public Vector3 deltaPosition;
        /// <summary>
        /// Gets the avatar delta rotation for the last evaluated frame.
        /// </summary>
        public Quaternion deltaRotation;
        /// <summary>
        /// Blends pivot point between body center of mass and feet pivot.
        /// </summary>
        public float feetPivotActive;
        /// <summary>
        /// Sets whether the Animator sends events of type AnimationEvent.
        /// </summary>
        public bool fireEvents;
        /// <summary>
        /// The current gravity weight based on current animations that are played.
        /// </summary>
        public float gravityWeight;
        /// <summary>
        /// Returns true if Animator has any playables assigned to it.
        /// </summary>
        public bool hasBoundPlayables;
        /// <summary>
        /// Returns true if the current rig has root motion.
        /// </summary>
        public bool hasRootMotion;
        /// <summary>
        /// Returns true if the object has a transform hierarchy.
        /// </summary>
        public bool hasTransformHierarchy;
        /// <summary>
        /// Returns the scale of the current Avatar for a humanoid rig, (1 by default if the rig is generic).
        /// </summary>
        public float humanScale;
        /// <summary>
        /// Returns true if the current rig is humanoid, false if it is generic.
        /// </summary>
        public bool isHuman;
        /// <summary>
        /// Returns whether the animator is initialized successfully.
        /// </summary>
        public bool isInitialized;
        /// <summary>
        /// If automatic matching is active.
        /// </summary>
        public bool isMatchingTarget;
        /// <summary>
        /// Returns true if the current rig is optimizable with AnimatorUtility.OptimizeTransformHierarchy.
        /// </summary>
        public bool isOptimizable;
        /// <summary>
        /// Controls the behaviour of the Animator component when a GameObject is disabled.
        /// </summary>
        public bool keepAnimatorControllerStateOnDisable;
        /// <summary>
        /// Returns the number of layers in the controller.
        /// </summary>
        public int layerCount;
        /// <summary>
        /// Additional layers affects the center of mass.
        /// </summary>
        public bool layersAffectMassCenter;
        /// <summary>
        /// Get left foot bottom height.
        /// </summary>
        public float leftFeetBottomHeight;
        /// <summary>
        /// Returns the number of parameters in the controller.
        /// </summary>
        public int parameterCount;
        /// <summary>
        /// The AnimatorControllerParameter list used by the animator. (Read Only)
        /// </summary>
        public readonly AnimatorControllerParameter[] parameters;
        /// <summary>
        /// Get the current position of the pivot.
        /// </summary>
        public Vector3 pivotPosition;
        /// <summary>
        /// Gets the pivot weight.
        /// </summary>
        public float pivotWeight;
        /// <summary>
        /// The PlayableGraph created by the Animator.
        /// </summary>
        public Playables.PlayableGraph playableGraph;
        /// <summary>
        /// Sets the playback position in the recording buffer.
        /// </summary>
        public float playbackTime;
        /// <summary>
        /// Gets the mode of the Animator recorder.
        /// </summary>
        public AnimatorRecorderMode recorderMode;
        /// <summary>
        /// Start time of the first frame of the buffer relative to the frame at which StartRecording was called.
        /// </summary>
        public float recorderStartTime;
        /// <summary>
        /// End time of the recorded clip relative to when StartRecording was called.
        /// </summary>
        public float recorderStopTime;
        /// <summary>
        /// Get right foot bottom height.
        /// </summary>
        public float rightFeetBottomHeight;
        /// <summary>
        /// The root position, the position of the game object.
        /// </summary>
        public Vector3 rootPosition;
        /// <summary>
        /// The root rotation, the rotation of the game object.
        /// </summary>
        public Quaternion rootRotation;
        /// <summary>
        /// The runtime representation of AnimatorController that controls the Animator.
        /// </summary>
        public RuntimeAnimatorController runtimeAnimatorController;
        /// <summary>
        /// The playback speed of the Animator. 1 is normal playback speed.
        /// </summary>
        public float speed;
        /// <summary>
        /// Automatic stabilization of feet during transition and blending.
        /// </summary>
        public bool stabilizeFeet;
        /// <summary>
        /// Returns the position of the target specified by SetTarget.
        /// </summary>
        public Vector3 targetPosition;
        /// <summary>
        /// Returns the rotation of the target specified by SetTarget.
        /// </summary>
        public Quaternion targetRotation;
        /// <summary>
        /// Specifies the update mode of the Animator.
        /// </summary>
        public AnimatorUpdateMode updateMode;
        /// <summary>
        /// Gets the avatar velocity for the last evaluated frame.
        /// </summary>
        public Vector3 velocity;


        /// <summary>
        /// Apply the default Root Motion.
        /// </summary>
        public void ApplyBuiltinRootMotion();
        /// <summary>
        /// Creates a crossfade from the current state to any other state using normalized times.
        /// </summary>
        /// <param name="stateName">The name of the state.</param>
        /// <param name="normalizedTransitionDuration">The duration of the transition (normalized).</param>
        /// <param name="layer">The layer where the crossfade occurs.</param>
        /// <param name="normalizedTimeOffset">The time of the state (normalized).</param>
        /// <param name="normalizedTransitionTime">The time of the transition (normalized).</param>
        public void CrossFade(string stateName, float normalizedTransitionDuration, int layer = -1, float normalizedTimeOffset = float.NegativeInfinity, float normalizedTransitionTime = 0.0f);
        public void CrossFade(int stateHashName, float normalizedTransitionDuration, int layer = -1, float normalizedTimeOffset = 0.0f, float normalizedTransitionTime = 0.0f);
        /// <summary>
        /// Creates a crossfade from the current state to any other state using times in seconds.
        /// </summary>
        /// <param name="stateHashName">The name of the state.</param>
        /// <param name="fixedTransitionDuration">The duration of the transition (in seconds).</param>
        /// <param name="layer">The layer where the crossfade occurs.</param>
        /// <param name="fixedTimeOffset">The time of the state (in seconds).</param>
        /// <param name="normalizedTransitionTime">The time of the transition (normalized).</param>
        public void CrossFadeInFixedTime(int stateHashName, float fixedTransitionDuration, int layer = -1, float fixedTimeOffset = 0.0f, float normalizedTransitionTime = 0.0f);
        public void CrossFadeInFixedTime(string stateName, float fixedTransitionDuration, int layer = -1, float fixedTimeOffset = 0.0f, float normalizedTransitionTime = 0.0f);
        /// <summary>
        /// Returns an AnimatorTransitionInfo with the informations on the current transition.
        /// </summary>
        /// <param name="layerIndex">The layer's index.</param>
        /// <returns>AnimatorTransitionInfo An AnimatorTransitionInfo with the informations on the current transition.</returns>
        public AnimatorTransitionInfo GetAnimatorTransitionInfo(int layerIndex);
        /// <summary>
        /// Returns the first StateMachineBehaviour that matches type T or is derived from T. Returns null if none are found.
        /// </summary>
        public T GetBehaviour<T>();
        /// <summary>
        /// Returns all StateMachineBehaviour that match type T or are derived from T. Returns null if none are found.
        /// </summary>
        public T[] GetBehaviours<T>();
        /// <summary>
        /// Returns Transform mapped to this human bone id.
        /// </summary>
        /// <param name="humanBoneId">The human bone that is queried, see enum HumanBodyBones for a list of possible values.</param>
        public Transform GetBoneTransform(HumanBodyBones humanBoneId);
        /// <summary>
        /// Returns the value of the given boolean parameter.
        /// </summary>
        /// <param name="name">The parameter name.</param>
        /// <returns>bool The value of the parameter.</returns>
        public bool GetBool(string name);
        public bool GetBool(int id);
        /// <summary>
        /// Returns an array of all the AnimatorClipInfo in the current state of the given layer.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <returns>AnimatorClipInfo[] An array of all the AnimatorClipInfo in the current state.</returns>
        public AnimatorClipInfo[] GetCurrentAnimatorClipInfo(int layerIndex);
        /// <summary>
        /// Returns the number of AnimatorClipInfo in the current state.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <returns>int The number of AnimatorClipInfo in the current state.</returns>
        public int GetCurrentAnimatorClipInfoCount(int layerIndex);
        /// <summary>
        /// Returns an AnimatorStateInfo with the information on the current state.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <returns>AnimatorStateInfo An AnimatorStateInfo with the information on the current state.</returns>
        public AnimatorStateInfo GetCurrentAnimatorStateInfo(int layerIndex);
        /// <summary>
        /// Returns the value of the given float parameter.
        /// </summary>
        /// <param name="name">The parameter name.</param>
        /// <returns>float The value of the parameter.</returns>
        public float GetFloat(string name);
        public float GetFloat(int id);
        /// <summary>
        /// Gets the position of an IK hint.
        /// </summary>
        /// <param name="hint">The AvatarIKHint that is queried.</param>
        /// <returns>Vector3 Return the current position of this IK hint in world space.</returns>
        public Vector3 GetIKHintPosition(AvatarIKHint hint);
        /// <summary>
        /// Gets the translative weight of an IK Hint (0 = at the original animation before IK, 1 = at the hint).
        /// </summary>
        /// <param name="hint">The AvatarIKHint that is queried.</param>
        /// <returns>float Return translative weight.</returns>
        public float GetIKHintPositionWeight(AvatarIKHint hint);
        /// <summary>
        /// Gets the position of an IK goal.
        /// </summary>
        /// <param name="goal">The AvatarIKGoal that is queried.</param>
        /// <returns>Vector3 Return the current position of this IK goal in world space.</returns>
        public Vector3 GetIKPosition(AvatarIKGoal goal);
        /// <summary>
        /// Gets the translative weight of an IK goal (0 = at the original animation before IK, 1 = at the goal).
        /// </summary>
        /// <param name="goal">The AvatarIKGoal that is queried.</param>
        public float GetIKPositionWeight(AvatarIKGoal goal);
        /// <summary>
        /// Gets the rotation of an IK goal.
        /// </summary>
        /// <param name="goal">The AvatarIKGoal that is is queried.</param>
        public Quaternion GetIKRotation(AvatarIKGoal goal);
        /// <summary>
        /// Gets the rotational weight of an IK goal (0 = rotation before IK, 1 = rotation at the IK goal).
        /// </summary>
        /// <param name="goal">The AvatarIKGoal that is queried.</param>
        public float GetIKRotationWeight(AvatarIKGoal goal);
        /// <summary>
        /// Returns the value of the given integer parameter.
        /// </summary>
        /// <param name="name">The parameter name.</param>
        /// <returns>int The value of the parameter.</returns>
        public int GetInteger(string name);
        public int GetInteger(int id);
        /// <summary>
        /// Returns the index of the layer with the given name.
        /// </summary>
        /// <param name="layerName">The layer name.</param>
        /// <returns>int The layer index.</returns>
        public int GetLayerIndex(string layerName);
        /// <summary>
        /// Returns the layer name.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <returns>string The layer name.</returns>
        public string GetLayerName(int layerIndex);
        /// <summary>
        /// Returns the weight of the layer at the specified index.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <returns>float The layer weight.</returns>
        public float GetLayerWeight(int layerIndex);
        /// <summary>
        /// Returns an array of all the AnimatorClipInfo in the next state of the given layer.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <returns>AnimatorClipInfo[] An array of all the AnimatorClipInfo in the next state.</returns>
        public AnimatorClipInfo[] GetNextAnimatorClipInfo(int layerIndex);
        /// <summary>
        /// Returns the number of AnimatorClipInfo in the next state.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <returns>int The number of AnimatorClipInfo in the next state.</returns>
        public int GetNextAnimatorClipInfoCount(int layerIndex);
        /// <summary>
        /// Returns an AnimatorStateInfo with the information on the next state.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <returns>AnimatorStateInfo An AnimatorStateInfo with the information on the next state.</returns>
        public AnimatorStateInfo GetNextAnimatorStateInfo(int layerIndex);
        /// <summary>
        /// [no description]
        /// </summary>
        /// <param name="index">[no description]</param>
        /// <returns>[no description]</returns>
        public AnimatorControllerParameter GetParameter(int index);
        /// <summary>
        /// Returns true if the state exists in this layer, false otherwise.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <param name="stateID">The state ID.</param>
        /// <returns>bool True if the state exists in this layer, false otherwise.</returns>
        public bool HasState(int layerIndex, int stateID);
        /// <summary>
        /// Interrupts the automatic target matching.
        /// </summary>
        public void InterruptMatchTarget();
        public void InterruptMatchTarget(bool completeMatch = true);
        /// <summary>
        /// Returns true if there is a transition on the given layer, false otherwise.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <returns>bool True if there is a transition on the given layer, false otherwise.</returns>
        public bool IsInTransition(int layerIndex);
        /// <summary>
        /// Returns true if the parameter is controlled by a curve, false otherwise.
        /// </summary>
        /// <param name="name">The parameter name.</param>
        /// <returns>bool True if the parameter is controlled by a curve, false otherwise.</returns>
        public bool IsParameterControlledByCurve(string name);
        public bool IsParameterControlledByCurve(int id);
        /// <summary>
        /// Automatically adjust the <c>GameObject</c> position and rotation.
        /// </summary>
        /// <param name="matchPosition">The position we want the body part to reach.</param>
        /// <param name="matchRotation">The rotation in which we want the body part to be.</param>
        /// <param name="targetBodyPart">The body part that is involved in the match.</param>
        /// <param name="weightMask">Structure that contains weights for matching position and rotation.</param>
        /// <param name="startNormalizedTime">Start time within the animation clip (0 - beginning of clip, 1 - end of clip).</param>
        /// <param name="targetNormalizedTime">End time within the animation clip (0 - beginning of clip, 1 - end of clip), values greater than 1 can be set to trigger a match after a certain number of loops. Ex: 2.3 means at 30% of 2nd loop.</param>
        public void MatchTarget(Vector3 matchPosition, Quaternion matchRotation, AvatarTarget targetBodyPart, MatchTargetWeightMask weightMask, float startNormalizedTime, float targetNormalizedTime = 1);
        /// <summary>
        /// Plays a state.
        /// </summary>
        /// <param name="stateName">The state name.</param>
        /// <param name="layer">The layer index. If layer is -1, it plays the first state with the given state name or hash.</param>
        /// <param name="normalizedTime">The time offset between zero and one.</param>
        public void Play(string stateName, int layer = -1, float normalizedTime = float.NegativeInfinity);
        public void Play(int stateNameHash, int layer = -1, float normalizedTime = float.NegativeInfinity);
        /// <summary>
        /// Plays a state.
        /// </summary>
        /// <param name="stateName">The state name.</param>
        /// <param name="layer">The layer index. If layer is -1, it plays the first state with the given state name or hash.</param>
        /// <param name="fixedTime">The time offset (in seconds).</param>
        public void PlayInFixedTime(string stateName, int layer = -1, float fixedTime = float.NegativeInfinity);
        public void PlayInFixedTime(int stateNameHash, int layer = -1, float fixedTime = float.NegativeInfinity);
        /// <summary>
        /// Rebind all the animated properties and mesh data with the Animator.
        /// </summary>
        public void Rebind();
        /// <summary>
        /// Resets the value of the given trigger parameter.
        /// </summary>
        /// <param name="name">The parameter name.</param>
        public void ResetTrigger(string name);
        public void ResetTrigger(int id);
        /// <summary>
        /// Sets local rotation of a human bone during a IK pass.
        /// </summary>
        /// <param name="humanBoneId">The human bone Id.</param>
        /// <param name="rotation">The local rotation.</param>
        public void SetBoneLocalRotation(HumanBodyBones humanBoneId, Quaternion rotation);
        /// <summary>
        /// Sets the value of the given boolean parameter.
        /// </summary>
        /// <param name="name">The parameter name.</param>
        /// <param name="value">The new parameter value.</param>
        public void SetBool(string name, bool value);
        public void SetBool(int id, bool value);
        /// <summary>
        /// Send float values to the Animator to affect transitions.
        /// </summary>
        /// <param name="name">The parameter name.</param>
        /// <param name="value">The new parameter value.</param>
        public void SetFloat(string name, float value);
        public void SetFloat(string name, float value, float dampTime, float deltaTime);
        public void SetFloat(int id, float value);
        public void SetFloat(int id, float value, float dampTime, float deltaTime);
        /// <summary>
        /// Sets the translative weight of an IK hint (0 = at the original animation before IK, 1 = at the hint).
        /// </summary>
        /// <param name="hint">The AvatarIKHint that is set.</param>
        /// <param name="value">The translative weight.</param>
        public void SetIKHintPositionWeight(AvatarIKHint hint, float value);
        /// <summary>
        /// Sets the position of an IK hint.
        /// </summary>
        /// <param name="hint">The AvatarIKHint that is set.</param>
        /// <param name="hintPosition">The position in world space.</param>
        public void SetIKHintPosition(AvatarIKHint hint, Vector3 hintPosition);
        /// <summary>
        /// Sets the position of an IK goal.
        /// </summary>
        /// <param name="goal">The AvatarIKGoal that is set.</param>
        /// <param name="goalPosition">The position in world space.</param>
        public void SetIKPosition(AvatarIKGoal goal, Vector3 goalPosition);
        /// <summary>
        /// Sets the translative weight of an IK goal (0 = at the original animation before IK, 1 = at the goal).
        /// </summary>
        /// <param name="goal">The AvatarIKGoal that is set.</param>
        /// <param name="value">The translative weight.</param>
        public void SetIKPositionWeight(AvatarIKGoal goal, float value);
        /// <summary>
        /// Sets the rotation of an IK goal.
        /// </summary>
        /// <param name="goal">The AvatarIKGoal that is set.</param>
        /// <param name="goalRotation">The rotation in world space.</param>
        public void SetIKRotation(AvatarIKGoal goal, Quaternion goalRotation);
        /// <summary>
        /// Sets the rotational weight of an IK goal (0 = rotation before IK, 1 = rotation at the IK goal).
        /// </summary>
        /// <param name="goal">The AvatarIKGoal that is set.</param>
        /// <param name="value">The rotational weight.</param>
        public void SetIKRotationWeight(AvatarIKGoal goal, float value);
        /// <summary>
        /// Sets the value of the given integer parameter.
        /// </summary>
        /// <param name="name">The parameter name.</param>
        /// <param name="value">The new parameter value.</param>
        public void SetInteger(string name, int value);
        public void SetInteger(int id, int value);
        /// <summary>
        /// Sets the weight of the layer at the given index.
        /// </summary>
        /// <param name="layerIndex">The layer index.</param>
        /// <param name="weight">The new layer weight.</param>
        public void SetLayerWeight(int layerIndex, float weight);
        /// <summary>
        /// Sets the look at position.
        /// </summary>
        /// <param name="lookAtPosition">The position to lookAt.</param>
        public void SetLookAtPosition(Vector3 lookAtPosition);
        /// <summary>
        /// Set look at weights.
        /// </summary>
        /// <param name="weight">(0-1) the global weight of the LookAt, multiplier for other parameters.</param>
        /// <param name="bodyWeight">(0-1) determines how much the body is involved in the LookAt.</param>
        /// <param name="headWeight">(0-1) determines how much the head is involved in the LookAt.</param>
        /// <param name="eyesWeight">(0-1) determines how much the eyes are involved in the LookAt.</param>
        /// <param name="clampWeight">
        /// (0-1) 0.0 means the character is completely unrestrained in motion,
        /// 1.0 means he's completely clamped (look at becomes impossible), and 0.5 means he'll be able to move on half of the possible range (180 degrees).
        /// </param>
        public void SetLookAtWeight(float weight, float bodyWeight = 0.0f, float headWeight = 1.0f, float eyesWeight = 0.0f, float clampWeight = 0.5f);
        /// <summary>
        /// Sets an AvatarTarget and a targetNormalizedTime for the current state.
        /// </summary>
        /// <param name="targetIndex">The avatar body part that is queried.</param>
        /// <param name="targetNormalizedTime">The current state Time that is queried.</param>
        public void SetTarget(AvatarTarget targetIndex, float targetNormalizedTime);
        /// <summary>
        /// Sets the value of the given trigger parameter.
        /// </summary>
        /// <param name="name">The parameter name.</param>
        public void SetTrigger(string name);
        public void SetTrigger(int id);
        /// <summary>
        /// Sets the animator in playback mode.
        /// </summary>
        public void StartPlayback();
        /// <summary>
        /// Sets the animator in recording mode, and allocates a circular buffer of size frameCount.
        /// </summary>
        /// <param name="frameCount">
        /// The number of frames (updates) that will be recorded.
        /// If frameCount is 0, the recording will continue until the user calls StopRecording. The maximum value for frameCount is 10000.
        /// </param>
        public void StartRecording(int frameCount);
        /// <summary>
        /// Stops the animator playback mode. When playback stops, the avatar resumes getting control from game logic.
        /// </summary>
        public void StopPlayback();
        /// <summary>
        /// Stops animator record mode.
        /// </summary>
        public void StopRecording();
        /// <summary>
        /// Evaluates the animator based on deltaTime.
        /// </summary>
        /// <param name="deltaTime">The time delta.</param>
        public void Update(float deltaTime);
        /// <summary>
        /// Forces a write of the default values stored in the animator.
        /// </summary>
        public void WriteDefaultValues();
        

        /// <summary>
        /// Generates an parameter id from a string.
        /// </summary>
        /// <param name="name">The string to convert to Id.</param>
        public static int StringToHash(string name);
    }

    /// <summary>
    /// Used to communicate between scripting and the controller.
    /// Some parameters can be set in scripting and used by the controller, 
    /// while other parameters are based on Custom Curves in Animation Clips and can be sampled using the scripting API.
    /// </summary>
    public class AnimatorControllerParameter {
        /// <summary>
        /// The default bool value for the parameter.
        /// </summary>
        public bool defaultBool;
        /// <summary>
        /// The default float value for the parameter.
        /// </summary>
        public float defaultFloat;
        /// <summary>
        /// The default int value for the parameter.
        /// </summary>
        public int defaultInt;
        /// <summary>
        /// The name of the parameter.
        /// </summary>
        public string name;
        /// <summary>
        /// Returns the hash of the parameter based on its name.
        /// </summary>
        public int nameHash;
        /// <summary>
        /// The type of the parameter.
        /// </summary>
        public AnimatorControllerParameterType type;
    }

    /// <summary>
    /// Avatar definition.
    /// </summary>
    public class Avatar : Object {
        /// <summary>
        /// Returns the HumanDescription used to create this Avatar.
        /// </summary>
        public HumanDescription humanDescription;
        /// <summary>
        /// Return true if this avatar is a valid human avatar.
        /// </summary>
        public bool isHuman;
        /// <summary>
        /// Return true if this avatar is a valid mecanim avatar. It can be a generic avatar or a human avatar.
        /// </summary>
        public bool isValid;
    }
}