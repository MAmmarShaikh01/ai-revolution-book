import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Physical AI Foundations for Robotics',
      items: [
        'module-1/physical-ai-foundations',
        'module-1/embodied-intelligence',
        'module-1/robot-dynamics',
        'module-1/simulation-environments',
        'module-1/sensing-and-perception-in-physical-systems',
        'module-1/the-embodied-mind-foundations-of-physical-intelligence',
        'module-1/morphological-computation-and-mechanical-intelligence',
        'module-1/the-reality-gap-simulation-vs-physical-embodiment',
        'module-1/actuation-theory-from-motors-to-muscles',
        'module-1/the-physics-of-intelligence-thermodynamics-and-information-processing',
        'module-1/morphological-development-and-evolutionary-physical-intelligence',
        'module-1/historical-perspectives-from-automata-to-physical-ai',
        'module-1/physical-ai-project',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Control Systems for Humanoid Robots',
      items: [
        'module-2/balance-locomotion',
        'module-2/motor-control',
        'module-2/trajectory-planning',
        'module-2/reinforcement-learning-in-continuous-physical-environments',
        'module-2/self-supervised-learning-in-physical-systems',
        'module-2/foundation-models-for-physical-systems',
        'module-2/multimodal-perception-and-learning',
        'module-2/world-models-and-environmental-dynamics',
        'module-2/simulation-to-reality-transfer-methods',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Perception & Sensing for Humanoids',
      items: [
        'module-3/bipedal-locomotion-and-walking-control',
        'module-3/manipulation-and-dexterous-control',
        'module-3/whole-body-control-for-humanoid-robots',
        'module-3/adaptive-control-and-learning-for-humanoid-systems',
        'module-3/energy-efficiency-and-power-management',
        'module-3/human-robot-interaction-in-humanoid-robotics',
        'module-3/safety-and-compliance-in-humanoid-robotics',
        'module-3/advanced-topics-in-humanoid-robotics-systems-and-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Human-Robot Interaction & Autonomy',
      items: [
        'module-4/deployed-humanoid-platforms-and-industry-applications',
        'module-4/failures-and-setbacks-in-humanoid-robotics',
        'module-4/natural-language-processing-for-robots',
        'module-4/social-robotics-principles',
        'module-4/cognitive-architectures',
        'module-4/capstone-project-autonomous-humanoid-behaviors',
      ],
    },
  ],
};

export default sidebars;
