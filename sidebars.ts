import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  bookSidebar: [
    {
      type: 'category',
      label: 'Quantum AI Book',
      collapsible: false,
      items: [
        'introduction-to-quantum-ai',
        'spec-driven-agents',
        'predictive-machine-architectures',
        'fastapi-mcp-integration',
        'future-of-ai-driven-book-development',
        'quantum-algorithms-for-machine-learning',
        'quantum-enhanced-optimization',
        'quantum-ai-applications',
        'quantum-ai-hardware-and-software',
        'future-of-quantum-ai',
      ],
    },
  ],
};

export default sidebars;
