/*
 * MaterialSetup.cs
 * Module 2: The Digital Twin - Chapter 2
 *
 * Utility for programmatically setting up PBR materials on imported robots.
 * Supports Unity's Universal Render Pipeline (URP) and Standard shader.
 */

using System.Collections.Generic;
using UnityEngine;

namespace DigitalTwin.Materials
{
    /// <summary>
    /// Material configuration for robot parts.
    /// </summary>
    [System.Serializable]
    public class RobotMaterialConfig
    {
        public string partNameContains;
        public Color baseColor = Color.gray;
        [Range(0, 1)] public float metallic = 0.5f;
        [Range(0, 1)] public float smoothness = 0.5f;
        public Color emissionColor = Color.black;
        [Range(0, 2)] public float emissionIntensity = 0f;
    }

    /// <summary>
    /// Handles material setup for imported robot models.
    /// </summary>
    public class MaterialSetup : MonoBehaviour
    {
        [Header("Material Configurations")]
        [Tooltip("Material configurations for different robot parts")]
        public List<RobotMaterialConfig> materialConfigs = new List<RobotMaterialConfig>
        {
            new RobotMaterialConfig { partNameContains = "torso", baseColor = new Color(0.2f, 0.2f, 0.25f), metallic = 0.8f, smoothness = 0.6f },
            new RobotMaterialConfig { partNameContains = "head", baseColor = new Color(0.3f, 0.3f, 0.35f), metallic = 0.9f, smoothness = 0.7f },
            new RobotMaterialConfig { partNameContains = "arm", baseColor = new Color(0.25f, 0.25f, 0.3f), metallic = 0.7f, smoothness = 0.5f },
            new RobotMaterialConfig { partNameContains = "leg", baseColor = new Color(0.25f, 0.25f, 0.3f), metallic = 0.7f, smoothness = 0.5f },
            new RobotMaterialConfig { partNameContains = "foot", baseColor = new Color(0.15f, 0.15f, 0.2f), metallic = 0.6f, smoothness = 0.4f },
            new RobotMaterialConfig { partNameContains = "sensor", baseColor = Color.black, metallic = 0.3f, smoothness = 0.8f, emissionColor = Color.cyan, emissionIntensity = 0.5f },
            new RobotMaterialConfig { partNameContains = "eye", baseColor = Color.black, metallic = 0.1f, smoothness = 0.9f, emissionColor = Color.blue, emissionIntensity = 1f }
        };

        [Header("Default Material")]
        [Tooltip("Default material for parts not matching any config")]
        public RobotMaterialConfig defaultConfig = new RobotMaterialConfig
        {
            partNameContains = "",
            baseColor = new Color(0.5f, 0.5f, 0.55f),
            metallic = 0.5f,
            smoothness = 0.5f
        };

        [Header("Shader Settings")]
        [Tooltip("Use URP Lit shader (false = Standard shader)")]
        public bool useURPShader = true;

        private Dictionary<string, Material> materialCache = new Dictionary<string, Material>();

        /// <summary>
        /// Apply materials to all renderers in the target object.
        /// </summary>
        public void ApplyMaterials(GameObject target)
        {
            if (target == null)
            {
                Debug.LogError("Target object is null");
                return;
            }

            Renderer[] renderers = target.GetComponentsInChildren<Renderer>();
            int appliedCount = 0;

            foreach (Renderer renderer in renderers)
            {
                RobotMaterialConfig config = FindMatchingConfig(renderer.gameObject.name);
                Material material = GetOrCreateMaterial(config, renderer.gameObject.name);

                renderer.material = material;
                appliedCount++;
            }

            Debug.Log($"Applied materials to {appliedCount} renderers on {target.name}");
        }

        /// <summary>
        /// Find material config matching the part name.
        /// </summary>
        private RobotMaterialConfig FindMatchingConfig(string partName)
        {
            string lowerName = partName.ToLower();

            foreach (RobotMaterialConfig config in materialConfigs)
            {
                if (!string.IsNullOrEmpty(config.partNameContains) &&
                    lowerName.Contains(config.partNameContains.ToLower()))
                {
                    return config;
                }
            }

            return defaultConfig;
        }

        /// <summary>
        /// Get cached material or create new one.
        /// </summary>
        private Material GetOrCreateMaterial(RobotMaterialConfig config, string partName)
        {
            string cacheKey = $"{config.partNameContains}_{config.baseColor}_{config.metallic}_{config.smoothness}";

            if (materialCache.TryGetValue(cacheKey, out Material cached))
            {
                return cached;
            }

            Material material = CreatePBRMaterial(config);
            material.name = $"Robot_{partName}_Material";
            materialCache[cacheKey] = material;

            return material;
        }

        /// <summary>
        /// Create a PBR material with the given configuration.
        /// </summary>
        private Material CreatePBRMaterial(RobotMaterialConfig config)
        {
            Shader shader;

            if (useURPShader)
            {
                shader = Shader.Find("Universal Render Pipeline/Lit");
                if (shader == null)
                {
                    Debug.LogWarning("URP Lit shader not found, falling back to Standard");
                    shader = Shader.Find("Standard");
                }
            }
            else
            {
                shader = Shader.Find("Standard");
            }

            Material material = new Material(shader);

            // Set base color
            if (material.HasProperty("_BaseColor"))
            {
                material.SetColor("_BaseColor", config.baseColor);
            }
            else if (material.HasProperty("_Color"))
            {
                material.SetColor("_Color", config.baseColor);
            }

            // Set metallic
            if (material.HasProperty("_Metallic"))
            {
                material.SetFloat("_Metallic", config.metallic);
            }

            // Set smoothness
            if (material.HasProperty("_Smoothness"))
            {
                material.SetFloat("_Smoothness", config.smoothness);
            }
            else if (material.HasProperty("_Glossiness"))
            {
                material.SetFloat("_Glossiness", config.smoothness);
            }

            // Set emission
            if (config.emissionIntensity > 0)
            {
                material.EnableKeyword("_EMISSION");
                Color emissionFinal = config.emissionColor * config.emissionIntensity;

                if (material.HasProperty("_EmissionColor"))
                {
                    material.SetColor("_EmissionColor", emissionFinal);
                }
            }

            return material;
        }

        /// <summary>
        /// Clear the material cache.
        /// </summary>
        public void ClearCache()
        {
            foreach (Material mat in materialCache.Values)
            {
                if (Application.isPlaying)
                {
                    Destroy(mat);
                }
                else
                {
                    DestroyImmediate(mat);
                }
            }
            materialCache.Clear();
        }

        private void OnDestroy()
        {
            ClearCache();
        }

#if UNITY_EDITOR
        /// <summary>
        /// Editor utility to apply materials to selected object.
        /// </summary>
        [ContextMenu("Apply Materials to Children")]
        private void EditorApplyMaterials()
        {
            ApplyMaterials(gameObject);
        }
#endif
    }
}
