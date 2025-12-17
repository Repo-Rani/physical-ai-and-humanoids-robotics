import React from "react";
import Features from "../features";
import Details from "../detail";
import Footer from "../footer";

const Card = ({ img, name, role, bio, items = [] }) => {
  const [visible, setVisible] = React.useState(false);
  const [position, setPosition] = React.useState({ x: 0, y: 0 });
  const divRef = React.useRef < HTMLDivElement > null;

  const handleMouseMove = (e) => {
    if (divRef.current) {
      const bounds = divRef.current.getBoundingClientRect();
      setPosition({ x: e.clientX - bounds.left, y: e.clientY - bounds.top });
    }
  };

  return (
    <div
      useRef={divRef}
      onMouseMove={handleMouseMove}
      onMouseEnter={() => setVisible(true)}
      onMouseLeave={() => setVisible(false)}
      className="relative w-80 h-96 rounded-xl p-0.5 bg-white backdrop-blur-md text-gray-800 overflow-hidden shadow-lg cursor-pointer"
    >
      {visible && (
        <div
          className="pointer-events-none blur-xl bg-gradient-to-r from-blue-400 via-yellow-500 to-orange-900 size-60 absolute z-0 transition-opacity duration-300"
          style={{ top: position.y - 120, left: position.x - 120 }}
        />
      )}

      <div className="relative z-10 bg-green-900 px-6 py-8 h-full w-full rounded-[10px] flex flex-col items-center justify-center text-center">
        <img
          src={img}
          alt="Profile Avatar"
          className="w-24 h-24 rounded-full shadow-md my-4"
        />
        <h2 className="text-2xl font-bold  mb-1   text-[var(--hb-black-text)]">
          {name}{" "}
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
            <svg
              className="size-7"
              aria-hidden="true"
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              fill="currentColor"
              viewBox="0 0 24 24"
            >
              <path
                fillRule="evenodd"
                d="M12.006 2a9.847 9.847 0 0 0-6.484 2.44 10.32 10.32 0 0 0-3.393 6.17 10.48 10.48 0 0 0 1.317 6.955 10.045 10.045 0 0 0 5.4 4.418c.504.095.683-.223.683-.494 0-.245-.01-1.052-.014-1.908-2.78.62-3.366-1.21-3.366-1.21a2.711 2.711 0 0 0-1.11-1.5c-.907-.637.07-.621.07-.621.317.044.62.163.885.346.266.183.487.426.647.71.135.253.318.476.538.655a2.079 2.079 0 0 0 2.37.196c.045-.52.27-1.006.635-1.37-2.219-.259-4.554-1.138-4.554-5.07a4.022 4.022 0 0 1 1.031-2.75 3.77 3.77 0 0 1 .096-2.713s.839-.275 2.749 1.05a9.26 9.26 0 0 1 5.004 0c1.906-1.325 2.74-1.05 2.74-1.05.37.858.406 1.828.101 2.713a4.017 4.017 0 0 1 1.029 2.75c0 3.939-2.339 4.805-4.564 5.058a2.471 2.471 0 0 1 .679 1.897c0 1.372-.012 2.477-.012 2.814 0 .272.18.592.687.492a10.05 10.05 0 0 0 5.388-4.421 10.473 10.473 0 0 0 1.313-6.948 10.32 10.32 0 0 0-3.39-6.165A9.847 9.847 0 0 0 12.007 2Z"
                clipRule="evenodd"
              />
            </svg>
          </a>
          <a
            href="#"
            target="_blank"
            className="hover:-translate-y-0.5 transition"
          >
            <svg
              className="size-7"
              aria-hidden="true"
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              fill="currentColor"
              viewBox="0 0 24 24"
            >
              <path
                fillRule="evenodd"
                d="M12.51 8.796v1.697a3.738 3.738 0 0 1 3.288-1.684c3.455 0 4.202 2.16 4.202 4.97V19.5h-3.2v-5.072c0-1.21-.244-2.766-2.128-2.766-1.827 0-2.139 1.317-2.139 2.676V19.5h-3.19V8.796h3.168ZM7.2 6.106a1.61 1.61 0 0 1-.988 1.483 1.595 1.595 0 0 1-1.743-.348A1.607 1.607 0 0 1 5.6 4.5a1.601 1.601 0 0 1 1.6 1.606Z"
                clipRule="evenodd"
              />
              <path d="M7.2 8.809H4V19.5h3.2V8.809Z" />
            </svg>
          </a>
          <a
            href="#"
            target="_blank"
            className="hover:-translate-y-0.5 transition"
          >
            <svg
              className="size-7"
              aria-hidden="true"
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              fill="currentColor"
              viewBox="0 0 24 24"
            >
              <path
                fillRule="evenodd"
                d="M22 5.892a8.178 8.178 0 0 1-2.355.635 4.074 4.074 0 0 0 1.8-2.235 8.343 8.343 0 0 1-2.605.981A4.13 4.13 0 0 0 15.85 4a4.068 4.068 0 0 0-4.1 4.038c0 .31.035.618.105.919A11.705 11.705 0 0 1 3.4 4.734a4.006 4.006 0 0 0 1.268 5.392 4.165 4.165 0 0 1-1.859-.5v.05A4.057 4.057 0 0 0 6.1 13.635a4.192 4.192 0 0 1-1.856.07 4.108 4.108 0 0 0 3.831 2.807A8.36 8.36 0 0 1 2 18.184 11.732 11.732 0 0 0 8.291 20 11.502 11.502 0 0 0 19.964 8.5c0-.177 0-.349-.012-.523A8.143 8.143 0 0 0 22 5.892Z"
                clipRule="evenodd"
              />
            </svg>
          </a>
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
          0% {
            transform: translateX(0%);
          }

          100% {
            transform: translateX(-50%);
          }
        }
      `}</style>
      <section className="relative flex flex-col items-center max-md:px-2 bg-[var(--hb-black)] text-white text-sm pb-28 pt-8 bg-[url('https://raw.githubusercontent.com/prebuiltui/prebuiltui/main/assets/hero/green-gradient-bg.svg')] bg-top bg-no-repeat">
        {/* COMMUNITY BADGE */}
        <div className="flex flex-wrap items-center justify-center p-1.5 px-4 mt-24 rounded-full border bg-[var(--ifm-color-primary-dark)] hover:bg-[var(--bg-color)] text-xs">
          <p className="">AI NATIVE TEXTBOOK Series</p>
        </div>

        {/* HEADING */}
        <h1 className="text-4xl md:text-6xl text-center font-semibold max-w-4xl mt-5 bg-[var(--hb-black-text)]  text-transparent bg-clip-text">
          AI Native Software Development
        </h1>

        {/* SUBTITLE */}
        <p className="text-[var(--text-color)] md:text-base max-md:px-2 text-center max-w-3xl mt-3">
          Colearning Agentic AI with Python and TypeScript – Spec Driven
          Reusable Intelligence
        </p>

        {/* CTA BUTTONS */}
        <div className="flex items-center gap-2 mt-8 text-sm">
          <button className="px-6 py-2.5 bg-[var(--ifm-color-primary)] hover:bg-green-700 transition rounded-full">
            Get Started
          </button>
          <button className="flex items-center gap-2 text-[var(--hb-black-text)] bg-white/10 border border-white/15 rounded-full px-6 py-2.5">
            Learn More →
          </button>
        </div>
      </section>
      <section className="relative flex flex-col items-center max-md:px-2 bg-[var(--hb-black)] text-white text-sm pb-28 pt-8 bg-[url('https://raw.githubusercontent.com/prebuiltui/prebuiltui/main/assets/hero/green-gradient-bg.svg')] bg-top bg-no-repeat">
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

      <section className=" ">
        <Features />
      </section>

      <section>
        <Details />
      </section>

      <section>
        <Footer />
      </section>
    </>
  );
}
