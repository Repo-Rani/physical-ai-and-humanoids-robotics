import React from "react";
import Features from "../features";
import Details from "../detail";
import Footer from "../footer";
import Link from "@docusaurus/Link";

const Card = ({ img, name, role, bio, items = [] }) => {
  const [visible, setVisible] = React.useState(false);
  const [position, setPosition] = React.useState({ x: 0, y: 0 });
  const divRef = React.useRef(null);

  const handleMouseMove = (e) => {
    if (divRef.current) {
      const bounds = divRef.current.getBoundingClientRect();
      setPosition({ x: e.clientX - bounds.left, y: e.clientY - bounds.top });
    }
  };

  return (
    <div
      ref={divRef}
      onMouseMove={handleMouseMove}
      onMouseEnter={() => setVisible(true)}
      onMouseLeave={() => setVisible(false)}
      className="relative w-80 h-96 rounded-xl p-0.5 bg-white backdrop-blur-md text-gray-800 overflow-hidden shadow-lg cursor-pointer"
    >
      {visible && (
        <div
          className="pointer-events-none blur-xl bg-gradient-to-r from-blue-400 via-yellow-500 to-orange-900 w-60 h-60 absolute z-0 transition-opacity duration-300"
          style={{ top: position.y - 120, left: position.x - 120 }}
        />
      )}

      <div className="relative z-10 bg-green-900 px-6 py-8 h-full w-full rounded-[10px] flex flex-col items-center justify-center text-center">
        <img
          src={img}
          alt="Profile Avatar"
          className="w-24 h-24 rounded-full shadow-md my-4"
        />
        <h2 className="text-2xl font-bold mb-1 text-[var(--hb-black-text)]">
          {name}
        </h2>
        <p className="text-sm text-green-500 font-medium mb-3">{role}</p>
        <p className="text-sm text-gray-300 mb-2 px-4">{bio}</p>

        <ul className="text-gray-400 text-xs space-y-1 px-4 text-center">
          {items.map((item, index) => (
            <li key={index} className="leading-tight">
              • {item}
            </li>
          ))}
        </ul>

        <div className="flex space-x-4 mb-1 mt-3 text-xl text-white">
          <a
            href="#"
            target="_blank"
            className="hover:-translate-y-0.5 transition"
          >
            {/* GitHub */}
            <svg
              className="w-7 h-7"
              aria-hidden="true"
              xmlns="http://www.w3.org/2000/svg"
              fill="currentColor"
              viewBox="0 0 24 24"
            >
              <path
                fillRule="evenodd"
                clipRule="evenodd"
                d="M12.006 2a9.847 9.847 0 0 0-6.484 2.44 10.32 10.32 0 0 0-3.393 6.17 10.48 10.48 0 0 0 1.317 6.955 10.045 10.045 0 0 0 5.4 4.418c.504.095.683-.223.683-.494 0-.245-.01-1.052-.014-1.908-2.78.62-3.366-1.21-3.366-1.21a2.711 2.711 0 0 0-1.11-1.5c-.907-.637.07-.621.07-.621.317.044.62.163.885.346.266.183.487.426.647.71.135.253.318.476.538.655a2.079 2.079 0 0 0 2.37.196c.045-.52.27-1.006.635-1.37-2.219-.259-4.554-1.138-4.554-5.07a4.022 4.022 0 0 1 1.031-2.75 3.77 3.77 0 0 1 .096-2.713s.839-.275 2.749 1.05a9.26 9.26 0 0 1 5.004 0c1.906-1.325 2.74-1.05 2.74-1.05.37.858.406 1.828.101 2.713a4.017 4.017 0 0 1 1.029 2.75c0 3.939-2.339 4.805-4.564 5.058a2.471 2.471 0 0 1 .679 1.897c0 1.372-.012 2.477-.012 2.814 0 .272.18.592.687.492a10.05 10.05 0 0 0 5.388-4.421 10.473 10.473 0 0 0 1.313-6.948 10.32 10.32 0 0 0-3.39-6.165A9.847 9.847 0 0 0 12.007 2Z"
              />
            </svg>
          </a>
          {/* Add other social links here, same structure */}
        </div>
      </div>
    </div>
  );
};

export default function HeroSection() {
  return (
    <>
      <style>{`
        .marquee-inner {
          animation: marqueeScroll linear infinite;
        }
        @keyframes marqueeScroll {
          0% { transform: translateX(0%); }
          100% { transform: translateX(-50%); }
        }
      `}</style>

      {/* Hero Section */}
      <section className="relative flex flex-col items-center max-md:px-2 bg-[var(--hb-black)] text-white text-sm pb-28 pt-8 bg-[url('https://raw.githubusercontent.com/prebuiltui/prebuiltui/main/assets/hero/green-gradient-bg.svg')] bg-top bg-no-repeat">
        <div className="flex flex-wrap items-center justify-center p-1.5 px-4 mt-24 rounded-full border bg-[var(--ifm-color-primary-dark)] hover:bg-[var(--bg-color)] text-xs">
          <p>AI NATIVE TEXTBOOK Series</p>
        </div>

        <h1 className="text-4xl md:text-6xl text-center font-semibold max-w-4xl mt-5 bg-[var(--hb-black-text)] text-transparent bg-clip-text">
          AI Native Software Development
        </h1>

        <p className="text-[var(--text-color)] md:text-base max-md:px-2 text-center max-w-3xl mt-3">
          Colearning Agentic AI with Python and TypeScript – Spec Driven
          Reusable Intelligence
        </p>

        <div className="flex items-center gap-2 mt-8 text-sm">
          <button className="px-6 py-2.5 bg-[var(--ifm-color-primary)] hover:bg-gray-300 transition rounded-full">
            <Link href="http://localhost:3000/docs/intro">Get Started</Link>
          </button>
          <button className="flex items-center gap-2 text-[var(--hb-black-text)] bg-white/10 border border-white/15 rounded-full px-6 py-2.5">
            <Link href="http://localhost:3000/docs/intro">Learn More →</Link>
          </button>
        </div>
      </section>

      {/* Cards Section */}
      <section className="relative flex flex-col items-center max-md:px-2 bg-[var(--hb-black)] text-white text-sm pb-28 pt-8">
        <div className="flex flex-wrap justify-center gap-8">
          <Card
            img="img/r1.jpeg"
            name="AI Assisted"
            role="AI as Helper"
            bio="AI enhances your productivity with code completion, debugging assistance, and documentation."
            items={[
              "Code completion & suggestions",
              "Bug detection & debugging",
              "Documentation generation",
            ]}
          />
          <Card
            img="img/r2.jpeg"
            name="AI Driven"
            role="AI as Co-Creator"
            bio="AI generates significant code from specifications. You act as architect, director, and reviewer."
            items={[
              "Code generation from specs",
              "Automated testing & optimization",
              "Architecture from requirements",
            ]}
          />
          <Card
            img="img/r3.jpeg"
            name="AI Native"
            role="AI IS the Software"
            bio="Applications architected around AI capabilities. LLMs and agents are core functional components."
            items={[
              "Natural language interfaces",
              "Intelligent automation & reasoning",
              "Agent orchestration systems",
            ]}
          />
        </div>
      </section>

      {/* Features */}
      <section>
        <Features />
      </section>

      {/* Details */}
      <section>
        <Details />
      </section>

      {/* Footer */}
      <section>
        <Footer />
      </section>
    </>
  );
}
