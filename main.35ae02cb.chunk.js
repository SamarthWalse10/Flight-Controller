(this["webpackJsonparduino-serial-plotter-webapp"] =
  this["webpackJsonparduino-serial-plotter-webapp"] || []).push([
  [0],
  {
    139: function (e, t, n) {},
    289: function (e, t, n) {
      "use strict";
      n.r(t);
      var a,
        o = n(0),
        i = n.n(o),
        l = n(23),
        r = n.n(l),
        c = (n(139), n(6)),
        s = n(132);
      function d(e) {
        return "string" == typeof e && ["", "\n", "\r", "\r\n"].includes(e);
      }
      !(function (e) {
        var t;
        !(function (e) {
          var t, n;
          !(function (e) {
            (e.SEND_MESSAGE = "SEND_MESSAGE"),
              (e.CHANGE_SETTINGS = "CHANGE_SETTINGS");
          })(t || (t = {})),
            (e.ClientCommand = t),
            (function (e) {
              e.ON_SETTINGS_DID_CHANGE = "ON_SETTINGS_DID_CHANGE";
            })(n || (n = {})),
            (e.MiddlewareCommand = n),
            (e.isClientCommandMessage = function (e) {
              return (
                !Array.isArray(e) &&
                "string" == typeof e.command &&
                Object.keys(t).includes(e.command)
              );
            }),
            (e.isMiddlewareCommandMessage = function (e) {
              return (
                !Array.isArray(e) &&
                "string" == typeof e.command &&
                Object.keys(n).includes(e.command)
              );
            }),
            (e.isDataMessage = function (e) {
              return Array.isArray(e);
            });
        })(t || (t = e.Protocol || (e.Protocol = {})));
      })(a || (a = {}));
      var u = [
          "#0072B2",
          "#D55E00",
          "#009E73",
          "#E69F00",
          "#CC79A7",
          "#56B4E9",
          "#F0E442",
          "#95A5A6",
        ],
        v = {},
        b = 0,
        g = n(25),
        m = n.p + "static/media/checkmark.345d2161.svg",
        C = n(8);
      function f(e) {
        var t = e.dataset,
          n = e.chartRef,
          a = Object(o.useState)(!t.hidden),
          i = Object(c.a)(a, 2),
          l = i[0],
          r = i[1];
        if (!t) return Object(C.jsx)(C.Fragment, {});
        var s = {
          backgroundColor: l ? t.borderColor.toString() : "",
          borderColor: t.borderColor.toString(),
        };
        return Object(C.jsxs)("label", {
          onClick: function () {
            l ? ((t.hidden = !0), r(!1)) : ((t.hidden = !1), r(!0)),
              null == n || n.update();
          },
          children: [
            Object(C.jsx)("span", {
              style: s,
              className: "checkbox",
              children: l && Object(C.jsx)("img", { src: m, alt: "" }),
            }),
            null == t ? void 0 : t.label,
          ],
        });
      }
      var j = n(121),
        p = n(122),
        h = n.n(p),
        O = n(3),
        S = n.n(O);
      function x(e) {
        var t,
          n,
          i,
          l = e.chartRef,
          r = e.pause,
          s = e.config,
          d = e.cubicInterpolationMode,
          u = e.wsSend,
          v = e.setPause,
          b = e.setInterpolate,
          m = Object(o.useRef)(null),
          p = Object(o.useState)(!1),
          O = Object(c.a)(p, 2),
          x = O[0],
          E = O[1],
          w = Object(o.useState)(!1),
          I = Object(c.a)(w, 2),
          N = I[0],
          k = I[1],
          M = function () {
            m.current && m.current.getClientWidth() < m.current.getScrollWidth()
              ? (E(!0),
                k(!0),
                0 === m.current.getScrollLeft() && E(!1),
                m.current.getScrollLeft() + m.current.getClientWidth() >=
                  m.current.getScrollWidth() && k(!1))
              : (E(!1), k(!1));
          };
        return (
          Object(o.useEffect)(
            function () {
              M();
            },
            [l, r, v, s]
          ),
          Object(o.useEffect)(function () {
            return (
              window.addEventListener("resize", M),
              function () {
                window.removeEventListener("resize", M);
              }
            );
          }, []),
          Object(C.jsxs)("div", {
            className: "legend",
            children: [
              Object(C.jsxs)("div", {
                className: "scroll-wrap",
                children: [
                  x &&
                    Object(C.jsx)("button", {
                      className: "scroll-button left",
                      onClick: function () {
                        var e;
                        null === (e = m.current) ||
                          void 0 === e ||
                          e.scrollLeft(m.current.getScrollLeft() - 100);
                      },
                      children: Object(C.jsx)("svg", {
                        width: "15",
                        height: "15",
                        viewBox: "0 0 15 15",
                        fill: "none",
                        xmlns: "http://www.w3.org/2000/svg",
                        children: Object(C.jsx)("path", {
                          d: "M7.5 14.9375C8.971 14.9375 10.409 14.5013 11.6321 13.6841C12.8551 12.8668 13.8084 11.7052 14.3714 10.3462C14.9343 8.98718 15.0816 7.49175 14.7946 6.04902C14.5076 4.60628 13.7993 3.28105 12.7591 2.24089C11.719 1.20074 10.3937 0.492387 8.95098 0.205409C7.50825 -0.0815684 6.01282 0.0657188 4.65379 0.628645C3.29477 1.19157 2.13319 2.14486 1.31594 3.36795C0.498701 4.59104 0.0624998 6.029 0.0624997 7.5C0.0624995 9.47255 0.846091 11.3643 2.24089 12.7591C3.63569 14.1539 5.52745 14.9375 7.5 14.9375ZM4.99781 7.12281L8.18531 3.93531C8.2347 3.88552 8.29345 3.846 8.35819 3.81903C8.42293 3.79205 8.49237 3.77817 8.5625 3.77817C8.63263 3.77817 8.70207 3.79205 8.7668 3.81903C8.83154 3.846 8.8903 3.88552 8.93969 3.93531C8.98948 3.9847 9.029 4.04346 9.05597 4.10819C9.08294 4.17293 9.09683 4.24237 9.09683 4.3125C9.09683 4.38263 9.08294 4.45207 9.05597 4.51681C9.029 4.58154 8.98948 4.6403 8.93969 4.68969L6.12406 7.5L8.93969 10.3103C9.03972 10.4103 9.09592 10.546 9.09592 10.6875C9.09592 10.829 9.03972 10.9647 8.93969 11.0647C8.83965 11.1647 8.70397 11.2209 8.5625 11.2209C8.42102 11.2209 8.28535 11.1647 8.18531 11.0647L4.99781 7.87719C4.94802 7.8278 4.9085 7.76904 4.88152 7.70431C4.85455 7.63957 4.84067 7.57013 4.84067 7.5C4.84067 7.42987 4.85455 7.36043 4.88152 7.29569C4.9085 7.23096 4.94802 7.1722 4.99781 7.12281Z",
                          fill: "#424242",
                        }),
                      }),
                    }),
                  Object(C.jsx)(j.Scrollbars, {
                    ref: m,
                    className: "scrollbar",
                    renderTrackVertical: function (e) {
                      return Object(C.jsx)(
                        "div",
                        Object(g.a)(
                          Object(g.a)({}, e),
                          {},
                          { className: "track" }
                        )
                      );
                    },
                    renderTrackHorizontal: function (e) {
                      return Object(C.jsx)(
                        "div",
                        Object(g.a)(
                          Object(g.a)({}, e),
                          {},
                          { className: "track" }
                        )
                      );
                    },
                    style: {
                      height: "29px",
                      marginRight: "17px",
                      marginLeft: "-5px",
                    },
                    onScroll: M,
                    children: Object(C.jsx)("div", {
                      className: "chart-names",
                      children:
                        null == l
                          ? void 0
                          : l.data.datasets.map(function (e, t) {
                              return Object(C.jsx)(
                                f,
                                { dataset: e, chartRef: l },
                                t
                              );
                            }),
                    }),
                  }),
                  N &&
                    Object(C.jsx)("button", {
                      className: "scroll-button right",
                      onClick: function () {
                        var e;
                        return null === (e = m.current) || void 0 === e
                          ? void 0
                          : e.scrollLeft(m.current.getScrollLeft() + 100);
                      },
                      children: Object(C.jsx)("svg", {
                        width: "15",
                        height: "15",
                        viewBox: "0 0 15 15",
                        fill: "none",
                        xmlns: "http://www.w3.org/2000/svg",
                        children: Object(C.jsx)("path", {
                          d: "M7.5 0.0625C6.029 0.0625 4.59104 0.498702 3.36795 1.31594C2.14486 2.13319 1.19158 3.29477 0.628649 4.65379C0.0657225 6.01282 -0.0815647 7.50825 0.205413 8.95098C0.49239 10.3937 1.20074 11.719 2.2409 12.7591C3.28105 13.7993 4.60629 14.5076 6.04902 14.7946C7.49175 15.0816 8.98718 14.9343 10.3462 14.3714C11.7052 13.8084 12.8668 12.8551 13.6841 11.6321C14.5013 10.409 14.9375 8.971 14.9375 7.5C14.9375 5.52745 14.1539 3.63569 12.7591 2.24089C11.3643 0.846091 9.47255 0.0625 7.5 0.0625ZM10.0022 7.87719L6.81469 11.0647C6.7653 11.1145 6.70655 11.154 6.64181 11.181C6.57707 11.2079 6.50763 11.2218 6.4375 11.2218C6.36737 11.2218 6.29793 11.2079 6.2332 11.181C6.16846 11.154 6.1097 11.1145 6.06032 11.0647C6.01052 11.0153 5.971 10.9565 5.94403 10.8918C5.91706 10.8271 5.90317 10.7576 5.90317 10.6875C5.90317 10.6174 5.91706 10.5479 5.94403 10.4832C5.971 10.4185 6.01052 10.3597 6.06032 10.3103L8.87594 7.5L6.06032 4.68969C5.96028 4.58965 5.90408 4.45397 5.90408 4.3125C5.90408 4.17103 5.96028 4.03535 6.06032 3.93531C6.16035 3.83528 6.29603 3.77908 6.4375 3.77908C6.57898 3.77908 6.71465 3.83528 6.81469 3.93531L10.0022 7.12281C10.052 7.1722 10.0915 7.23096 10.1185 7.29569C10.1454 7.36043 10.1593 7.42987 10.1593 7.5C10.1593 7.57013 10.1454 7.63957 10.1185 7.70431C10.0915 7.76904 10.052 7.8278 10.0022 7.87719Z",
                          fill: "#2C353A",
                        }),
                      }),
                    }),
                ],
              }),
              Object(C.jsxs)("div", {
                className: "actions",
                children: [
                  Object(C.jsxs)("label", {
                    className: "interpolate",
                    children: [
                      Object(C.jsx)("span", { children: "Interpolate" }),
                      Object(C.jsx)(h.a, {
                        checkedIcon: !1,
                        uncheckedIcon: !1,
                        height: 20,
                        width: 37,
                        handleDiameter: 14,
                        offColor: "#C9D2D2",
                        onColor: "#008184",
                        onChange: function (e) {
                          b(e),
                            u({
                              command: a.Protocol.ClientCommand.CHANGE_SETTINGS,
                              data: { monitorUISettings: { interpolate: e } },
                            });
                        },
                        checked: "monotone" === d,
                      }),
                    ],
                  }),
                  Object(C.jsx)("button", {
                    disabled: !(null == s ||
                    null === (t = s.monitorUISettings) ||
                    void 0 === t
                      ? void 0
                      : t.connected),
                    className: S()("pause-button", { paused: r }),
                    title: (
                      null == s ||
                      null === (n = s.monitorUISettings) ||
                      void 0 === n
                        ? void 0
                        : n.connected
                    )
                      ? void 0
                      : "Serial disconnected",
                    onClick: function () {
                      var e;
                      (null == s ||
                      null === (e = s.monitorUISettings) ||
                      void 0 === e
                        ? void 0
                        : e.connected) && v(!r);
                    },
                    children:
                      r ||
                      !(null == s ||
                      null === (i = s.monitorUISettings) ||
                      void 0 === i
                        ? void 0
                        : i.connected)
                        ? "RUN"
                        : "STOP",
                  }),
                  Object(C.jsx)("button", {
                    className: "clear-button",
                    onClick: function () {
                      if (l && Array.isArray(l.data.datasets))
                        for (var e in null == l ? void 0 : l.data.datasets)
                          l.data.datasets[e].data = [];
                      null == l || l.update();
                    },
                    children: Object(C.jsxs)("svg", {
                      width: "24",
                      height: "24",
                      viewBox: "0 0 24 24",
                      fill: "none",
                      xmlns: "http://www.w3.org/2000/svg",
                      children: [
                        Object(C.jsx)("path", {
                          d: "M20.25 10.5H13.5C13.3011 10.5 13.1103 10.421 12.9697 10.2803C12.829 10.1397 12.75 9.94891 12.75 9.75C12.75 9.55109 12.829 9.36032 12.9697 9.21967C13.1103 9.07902 13.3011 9 13.5 9H20.25C20.4489 9 20.6397 9.07902 20.7803 9.21967C20.921 9.36032 21 9.55109 21 9.75C21 9.94891 20.921 10.1397 20.7803 10.2803C20.6397 10.421 20.4489 10.5 20.25 10.5Z",
                          fill: "#4E5B61",
                        }),
                        Object(C.jsx)("path", {
                          d: "M20.25 6H13.5C13.3011 6 13.1103 5.92098 12.9697 5.78033C12.829 5.63968 12.75 5.44891 12.75 5.25C12.75 5.05109 12.829 4.86032 12.9697 4.71967C13.1103 4.57902 13.3011 4.5 13.5 4.5H20.25C20.4489 4.5 20.6397 4.57902 20.7803 4.71967C20.921 4.86032 21 5.05109 21 5.25C21 5.44891 20.921 5.63968 20.7803 5.78033C20.6397 5.92098 20.4489 6 20.25 6Z",
                          fill: "#4E5B61",
                        }),
                        Object(C.jsx)("path", {
                          d: "M20.25 15H3.75C3.55109 15 3.36032 14.921 3.21967 14.7803C3.07902 14.6397 3 14.4489 3 14.25C3 14.0511 3.07902 13.8603 3.21967 13.7197C3.36032 13.579 3.55109 13.5 3.75 13.5H20.25C20.4489 13.5 20.6397 13.579 20.7803 13.7197C20.921 13.8603 21 14.0511 21 14.25C21 14.4489 20.921 14.6397 20.7803 14.7803C20.6397 14.921 20.4489 15 20.25 15Z",
                          fill: "#4E5B61",
                        }),
                        Object(C.jsx)("path", {
                          d: "M20.25 19.5H3.75C3.55109 19.5 3.36032 19.421 3.21967 19.2803C3.07902 19.1397 3 18.9489 3 18.75C3 18.5511 3.07902 18.3603 3.21967 18.2197C3.36032 18.079 3.55109 18 3.75 18H20.25C20.4489 18 20.6397 18.079 20.7803 18.2197C20.921 18.3603 21 18.5511 21 18.75C21 18.9489 20.921 19.1397 20.7803 19.2803C20.6397 19.421 20.4489 19.5 20.25 19.5Z",
                          fill: "#4E5B61",
                        }),
                        Object(C.jsx)("path", {
                          d: "M10.2829 9.9674C10.3532 10.0371 10.409 10.1201 10.4471 10.2115C10.4852 10.3029 10.5048 10.4009 10.5048 10.4999C10.5048 10.5989 10.4852 10.6969 10.4471 10.7883C10.409 10.8797 10.3532 10.9627 10.2829 11.0324C10.2132 11.1027 10.1303 11.1585 10.0389 11.1966C9.94748 11.2346 9.84945 11.2542 9.75044 11.2542C9.65143 11.2542 9.5534 11.2346 9.46201 11.1966C9.37062 11.1585 9.28766 11.1027 9.21794 11.0324L6.75044 8.5649L4.28294 11.0324C4.21322 11.1027 4.13027 11.1585 4.03888 11.1966C3.94748 11.2346 3.84945 11.2542 3.75044 11.2542C3.65143 11.2542 3.5534 11.2346 3.46201 11.1966C3.37062 11.1585 3.28766 11.1027 3.21794 11.0324C3.14765 10.9627 3.09185 10.8797 3.05377 10.7883C3.0157 10.6969 2.99609 10.5989 2.99609 10.4999C2.99609 10.4009 3.0157 10.3029 3.05377 10.2115C3.09185 10.1201 3.14765 10.0371 3.21794 9.9674L5.68544 7.4999L3.21794 5.0324C3.07671 4.89117 2.99737 4.69962 2.99737 4.4999C2.99737 4.30017 3.07671 4.10862 3.21794 3.96739C3.35917 3.82617 3.55072 3.74683 3.75044 3.74683C3.95017 3.74683 4.14171 3.82617 4.28294 3.96739L6.75044 6.4349L9.21794 3.96739C9.28787 3.89747 9.37089 3.842 9.46226 3.80415C9.55362 3.76631 9.65155 3.74683 9.75044 3.74683C9.84934 3.74683 9.94726 3.76631 10.0386 3.80415C10.13 3.842 10.213 3.89747 10.2829 3.96739C10.3529 4.03732 10.4083 4.12034 10.4462 4.21171C10.484 4.30307 10.5035 4.401 10.5035 4.4999C10.5035 4.59879 10.484 4.69672 10.4462 4.78808C10.4083 4.87945 10.3529 4.96247 10.2829 5.0324L7.81544 7.4999L10.2829 9.9674Z",
                          fill: "#4E5B61",
                        }),
                      ],
                    }),
                  }),
                ],
              }),
            ],
          })
        );
      }
      var E = n(15),
        w = (n(160), n(131)),
        I = n(89),
        N = i.a.memo(function (e) {
          var t,
            n,
            i,
            l,
            r,
            s,
            u,
            v = e.config,
            b = e.wsSend,
            m = Object(o.useState)(""),
            f = Object(c.a)(m, 2),
            j = f[0],
            p = f[1],
            h =
              null == v ||
              null === (t = v.pluggableMonitorSettings) ||
              void 0 === t ||
              null === (n = t.baudrate) ||
              void 0 === n
                ? void 0
                : n.selectedValue,
            O = !(null == v ||
            null === (i = v.monitorUISettings) ||
            void 0 === i
              ? void 0
              : i.connected),
            S =
              null == v || null === (l = v.monitorUISettings) || void 0 === l
                ? void 0
                : l.lineEnding,
            x = [
              { value: "", label: "No Line Ending" },
              { value: "\n", label: "New Line" },
              { value: "\r", label: "Carriage Return" },
              { value: "\r\n", label: "Both NL & CR" },
            ],
            E =
              null == v ||
              null === (r = v.pluggableMonitorSettings) ||
              void 0 === r ||
              null === (s = r.baudrate) ||
              void 0 === s ||
              null === (u = s.values) ||
              void 0 === u
                ? void 0
                : u.map(function (e) {
                    return { value: e, label: "".concat(e, " baud") };
                  });
          return Object(C.jsxs)("div", {
            className: "message-to-board",
            children: [
              Object(C.jsxs)("form", {
                className: "message-container",
                onSubmit: function (e) {
                  b({
                    command: a.Protocol.ClientCommand.SEND_MESSAGE,
                    data: j + S,
                  }),
                    p(""),
                    e.preventDefault(),
                    e.stopPropagation();
                },
                children: [
                  Object(C.jsx)("input", {
                    className: "message-to-board-input",
                    type: "text",
                    disabled: O,
                    value: j,
                    onChange: function (e) {
                      return p(e.target.value);
                    },
                    placeholder: "Type Message",
                  }),
                  Object(C.jsx)("button", {
                    type: "submit",
                    className: "message-to-board-send-button",
                    disabled: 0 === j.length || O,
                    children: "Send",
                  }),
                  Object(C.jsx)(I.a, {
                    className: "singleselect lineending",
                    classNamePrefix: "select",
                    isDisabled: O,
                    value:
                      x[
                        x.findIndex(function (e) {
                          return e.value === S;
                        })
                      ],
                    name: "lineending",
                    options: x,
                    menuPlacement: "top",
                    onChange: function (e) {
                      e &&
                        d(e.value) &&
                        b({
                          command: a.Protocol.ClientCommand.CHANGE_SETTINGS,
                          data: { monitorUISettings: { lineEnding: e.value } },
                        });
                    },
                  }),
                ],
              }),
              Object(C.jsx)("div", {
                children: Object(C.jsx)("div", {
                  className: "baud",
                  children:
                    E &&
                    Object(C.jsx)(I.a, {
                      className: "singleselect",
                      classNamePrefix: "select",
                      isDisabled: O,
                      value:
                        E[
                          E.findIndex(function (e) {
                            return e.value === h;
                          })
                        ],
                      name: "baudrate",
                      options: E,
                      menuPlacement: "top",
                      onChange: function (e) {
                        var t;
                        e &&
                          b({
                            command: a.Protocol.ClientCommand.CHANGE_SETTINGS,
                            data: {
                              pluggableMonitorSettings: {
                                baudrate: Object(g.a)(
                                  Object(g.a)(
                                    {},
                                    null == v ||
                                      null ===
                                        (t = v.pluggableMonitorSettings) ||
                                      void 0 === t
                                      ? void 0
                                      : t.baudrate
                                  ),
                                  {},
                                  { selectedValue: e.value }
                                ),
                              },
                            },
                          });
                      },
                    }),
                }),
              }),
            ],
          });
        }),
        k = n(130);
      E.a.register(w.a);
      var M = new (function () {
        return new Worker(
          n.p + "static/js/msgAggregatorWorker.a9dd697c.worker.js"
        );
      })();
      var y = i.a.forwardRef(function (e, t) {
          var n,
            a,
            i,
            l,
            r,
            d,
            g,
            m,
            f = e.config,
            j = e.wsSend,
            p = Object(o.useRef)(),
            h = Object(o.useState)(0),
            O = Object(c.a)(h, 2)[1],
            S = Object(o.useState)(!1),
            E = Object(c.a)(S, 2),
            w = E[0],
            I = E[1],
            y = Object(o.useState)(
              null == f || null === (n = f.monitorUISettings) || void 0 === n
                ? void 0
                : n.connected
            ),
            L = Object(c.a)(y, 2),
            A = L[0],
            T = L[1],
            U = Object(o.useState)(1000),
            P = Object(c.a)(U, 1)[0],
            D = Object(o.useState)(
              (
                null == f || null === (a = f.monitorUISettings) || void 0 === a
                  ? void 0
                  : a.interpolate
              )
                ? "monotone"
                : "default"
            ),
            H = Object(c.a)(D, 2),
            G = H[0],
            R = H[1],
            _ = Object(o.useState)({ datasets: [] }),
            B = Object(c.a)(_, 1)[0],
            F = Object(o.useState)({
              animation: !1,
              maintainAspectRatio: !1,
              normalized: !0,
              parsing: !1,
              datasets: { line: { pointRadius: 0, pointHoverRadius: 0 } },
              interaction: { intersect: !1 },
              plugins: {
                tooltip: {
                  caretPadding: 9,
                  enabled: !1,
                  bodyFont: { family: "Open Sans" },
                  titleFont: { family: "Open Sans" },
                },
                decimation: { enabled: !0, algorithm: "min-max" },
                legend: { display: !1 },
              },
              elements: { line: { tension: 0 } },
              scales: {
                y: {
                  grid: {
                    color: (
                      null == f ||
                      null === (i = f.monitorUISettings) ||
                      void 0 === i
                        ? void 0
                        : i.darkTheme
                    )
                      ? "#2C353A"
                      : "#ECF1F1",
                  },
                  ticks: {
                    color: (
                      null == f ||
                      null === (l = f.monitorUISettings) ||
                      void 0 === l
                        ? void 0
                        : l.darkTheme
                    )
                      ? "#DAE3E3"
                      : "#2C353A",
                    font: { family: "Open Sans" },
                  },
                  grace: "5%",
                },
                x: {
                  grid: {
                    color: (
                      null == f ||
                      null === (r = f.monitorUISettings) ||
                      void 0 === r
                        ? void 0
                        : r.darkTheme
                    )
                      ? "#2C353A"
                      : "#ECF1F1",
                  },
                  display: !0,
                  ticks: {
                    font: { family: "Open Sans" },
                    color: (
                      null == f ||
                      null === (d = f.monitorUISettings) ||
                      void 0 === d
                        ? void 0
                        : d.darkTheme
                    )
                      ? "#DAE3E3"
                      : "#2C353A",
                    count: 5,
                    callback: function (e) {
                      return parseInt(e.toString(), 10);
                    },
                    align: "center",
                  },
                  type: "linear",
                  bounds: "data",
                },
              },
            }),
            W = Object(c.a)(F, 2),
            Z = W[0],
            V = W[1],
            z = Object(o.useCallback)(
              function (e) {
                var t;
                (Z.plugins.tooltip.enabled = e),
                  (Z.datasets.line.pointHoverRadius = e ? 3 : 0),
                  V(Z),
                  null === (t = p.current) || void 0 === t || t.update();
              },
              [Z]
            );
          return (
            Object(o.useEffect)(
              function () {
                var e, t;
                if (
                  !(null == f ||
                  null === (e = f.monitorUISettings) ||
                  void 0 === e
                    ? void 0
                    : e.connected)
                )
                  return T(!1), void z(!0);
                !A &&
                  (null == f ||
                  null === (t = f.monitorUISettings) ||
                  void 0 === t
                    ? void 0
                    : t.connected) &&
                  (M.postMessage({ command: "cleanup" }), T(!0), z(w));
              },
              [
                null == f || null === (g = f.monitorUISettings) || void 0 === g
                  ? void 0
                  : g.connected,
                A,
                w,
                z,
              ]
            ),
            Object(o.useImperativeHandle)(t, function () {
              return {
                addNewData: function (e) {
                  w || M.postMessage({ message: e });
                },
              };
            }),
            Object(o.useEffect)(
              function () {
                var e = function (e) {
                  !(function (e, t, n, a, o, i) {
                    if (t && t && t.data.datasets) {
                      var l = e.datasetNames,
                        r = e.parsedLines,
                        c = Object.keys(v);
                      c.length < 8 &&
                        l.forEach(function (e) {
                          if (!c.includes(e) && c.length < 8) {
                            var n = {
                              data: [],
                              label: e,
                              borderColor: u[c.length],
                              backgroundColor: u[c.length],
                              borderWidth: 1,
                              pointRadius: 0,
                              cubicInterpolationMode: a,
                            };
                            (v[e] = n),
                              t.data.datasets.push(n),
                              c.push(e),
                              i(c.length);
                          }
                        }),
                        r.forEach(function (e) {
                          var t,
                            a =
                              "realtime" ===
                              (null === (t = n.scales.x) || void 0 === t
                                ? void 0
                                : t.type)
                                ? Date.now()
                                : b++;
                          Object.keys(v).forEach(function (t) {
                            var n = t in e ? { x: a, y: e[t] } : null;
                            n && v[t].data.push(n);
                          });
                        });
                      for (
                        var s = b - o, d = 0;
                        d < t.data.datasets.length;
                        d++
                      )
                        for (
                          var g = t.data.datasets[d], m = 0, C = 0;
                          C < g.data.length;
                          C++
                        ) {
                          if (!(g.data[C] && g.data[C].x < s)) {
                            g.data.splice(0, m);
                            break;
                          }
                          m++,
                            g.data.length === m &&
                              (delete v[g.label],
                              t.data.datasets.splice(d, 1),
                              i(-1));
                        }
                      t.update();
                    }
                  })(e.data, p.current, Z, G, P, O);
                };
                return (
                  M.addEventListener("message", e),
                  function () {
                    M.removeEventListener("message", e);
                  }
                );
              },
              [G, Z, P]
            ),
            Object(C.jsx)(C.Fragment, {
              children: Object(C.jsxs)("div", {
                className: "chart-container",
                children: [
                  Object(C.jsx)(x, {
                    chartRef: p.current,
                    pause: w,
                    config: f,
                    cubicInterpolationMode: G,
                    wsSend: j,
                    setPause: function (e) {
                      var t;
                      e !== w &&
                        ("realtime" ===
                          (null === (t = Z.scales.x) || void 0 === t
                            ? void 0
                            : t.type) &&
                          (p.current.options.scales.x.realtime.pause = w),
                        I(e),
                        M.postMessage({ command: "cleanup" }),
                        z(e));
                    },
                    setInterpolate: function (e) {
                      var t = e ? "monotone" : "default";
                      if (p && p.current) {
                        for (
                          var n = 0;
                          n < p.current.data.datasets.length;
                          n++
                        ) {
                          var a = p.current.data.datasets[n];
                          a && (a.cubicInterpolationMode = t);
                        }
                        p.current.update(), R(t);
                      }
                    },
                  }),
                  Object(C.jsx)("div", {
                    className: "canvas-container",
                    children: Object(C.jsx)(s.a, {
                      data: B,
                      ref: p,
                      options: Z,
                    }),
                  }),
                  Object(C.jsx)(N, { config: f, wsSend: j }),
                  !A &&
                    Object(C.jsx)(k.a, {
                      anchorOrigin: {
                        horizontal: "center",
                        vertical: "bottom",
                      },
                      autoHideDuration: 7e3,
                      className: "snackbar",
                      closeable: !0,
                      isOpen: !0,
                      message: "Board disconnected",
                      theme: (
                        null == f ||
                        null === (m = f.monitorUISettings) ||
                        void 0 === m
                          ? void 0
                          : m.darkTheme
                      )
                        ? "dark"
                        : "light",
                      turnOffAutoHide: !0,
                    }),
                ],
              }),
            })
          );
        }),
        L = (n(7), n(45)),
        A = n.n(L),
        T = n(13),
        U = A.a.mark(H),
        P = function (e) {
          var t = "name ".concat(e),
            n = "".concat(Math.floor(10 * Math.random()));
          return "".concat(t, ":").concat(n);
        },
        D = function () {
          for (var e = [], t = 1; t <= 9; t++) {
            var n = P(t);
            e.push(n);
          }
          return [e.join(",") + "\n"];
        };
      function H() {
        var e;
        return A.a.wrap(function (t) {
          for (;;)
            switch ((t.prev = t.next)) {
              case 0:
                e = 0;
              case 1:
                if (!(++e > 9)) {
                  t.next = 8;
                  break;
                }
                return (t.next = 6), [1, 3];
              case 6:
                t.next = 10;
                break;
              case 8:
                return (t.next = 10), [1, 2, 3];
              case 10:
                t.next = 1;
                break;
              case 12:
              case "end":
                return t.stop();
            }
        }, U);
      }
      H();
      r.a.render(
        Object(C.jsx)(i.a.StrictMode, {
          children: Object(C.jsx)(function () {
            var e = Object(o.useState)(null),
              t = Object(c.a)(e, 2),
              n = t[0],
              i = t[1],
              l = Object(o.useState)(),
              r = Object(c.a)(l, 2),
              s = r[0],
              u = r[1],
              v = Object(o.useState)(),
              b = Object(c.a)(v, 2),
              g = b[0],
              m = b[1],
              f = Object(o.useState)(!1),
              j = Object(c.a)(f, 2),
              p = j[0],
              h = j[1],
              O = Object(o.useRef)(null),
              S = Object(o.useRef)(),
              x = Object(o.useCallback)(function (e) {
                var t;
                (null === (t = O.current) || void 0 === t
                  ? void 0
                  : t.readyState) === WebSocket.OPEN &&
                  O.current.send(JSON.stringify(e));
              }, []),
              E = Object(o.useCallback)(
                function (e) {
                  if (a.Protocol.isDataMessage(e))
                    S && S.current && S.current.addNewData(e);
                  else if (a.Protocol.isMiddlewareCommandMessage(e)) {
                    var t,
                      n = e.data.monitorUISettings || {},
                      o = n.autoscroll,
                      l = n.timestamp,
                      r = n.lineEnding,
                      c = n.interpolate,
                      s = n.darkTheme,
                      d = n.wsPort,
                      v = n.serialPort,
                      b = n.connected,
                      C = n.generate,
                      f = !1,
                      j = g;
                    void 0 !== v && ((j = v), (f = !0));
                    var p = !1 === b ? " (disconnected)" : "";
                    void 0 !== b &&
                      ((p = !1 === b ? " (disconnected)" : ""), (f = !0)),
                      f && (document.title = "".concat(j).concat(p)),
                      void 0 !== s &&
                        (s
                          ? document.body.classList.add("dark")
                          : document.body.classList.remove("dark"));
                    var h =
                        (null === (t = e.data.pluggableMonitorSettings) ||
                        void 0 === t
                          ? void 0
                          : t.baudrate) || {},
                      O = h.id,
                      x = h.label,
                      E = h.type,
                      w = h.values,
                      I = h.selectedValue;
                    i(function (e) {
                      var t, n, a, i, d, u, v, g, m, f, j, p, h, S, N, k, M;
                      return {
                        pluggableMonitorSettings: {
                          baudrate: {
                            id:
                              void 0 === O
                                ? null == e ||
                                  null === (t = e.pluggableMonitorSettings) ||
                                  void 0 === t ||
                                  null === (n = t.baudrate) ||
                                  void 0 === n
                                  ? void 0
                                  : n.id
                                : O,
                            label:
                              void 0 === x
                                ? null == e ||
                                  null === (a = e.pluggableMonitorSettings) ||
                                  void 0 === a ||
                                  null === (i = a.baudrate) ||
                                  void 0 === i
                                  ? void 0
                                  : i.label
                                : x,
                            type:
                              void 0 === E
                                ? null == e ||
                                  null === (d = e.pluggableMonitorSettings) ||
                                  void 0 === d ||
                                  null === (u = d.baudrate) ||
                                  void 0 === u
                                  ? void 0
                                  : u.type
                                : E,
                            values:
                              void 0 === w
                                ? null == e ||
                                  null === (v = e.pluggableMonitorSettings) ||
                                  void 0 === v ||
                                  null === (g = v.baudrate) ||
                                  void 0 === g
                                  ? void 0
                                  : g.values
                                : w,
                            selectedValue:
                              void 0 === I
                                ? (null == e ||
                                  null === (m = e.pluggableMonitorSettings) ||
                                  void 0 === m ||
                                  null === (f = m.baudrate) ||
                                  void 0 === f
                                    ? void 0
                                    : f.selectedValue) || "9600"
                                : I,
                          },
                        },
                        monitorUISettings: {
                          autoscroll:
                            void 0 === o
                              ? null == e ||
                                null === (j = e.monitorUISettings) ||
                                void 0 === j
                                ? void 0
                                : j.autoscroll
                              : o,
                          timestamp:
                            void 0 === l
                              ? null == e ||
                                null === (p = e.monitorUISettings) ||
                                void 0 === p
                                ? void 0
                                : p.timestamp
                              : l,
                          lineEnding:
                            void 0 === r
                              ? null == e ||
                                null === (h = e.monitorUISettings) ||
                                void 0 === h
                                ? void 0
                                : h.lineEnding
                              : r,
                          interpolate:
                            void 0 === c
                              ? null == e ||
                                null === (S = e.monitorUISettings) ||
                                void 0 === S
                                ? void 0
                                : S.interpolate
                              : c,
                          darkTheme:
                            void 0 === s
                              ? null == e ||
                                null === (N = e.monitorUISettings) ||
                                void 0 === N
                                ? void 0
                                : N.darkTheme
                              : s,
                          connected:
                            void 0 === b
                              ? null == e ||
                                null === (k = e.monitorUISettings) ||
                                void 0 === k
                                ? void 0
                                : k.connected
                              : b,
                          generate:
                            void 0 === C
                              ? null == e ||
                                null === (M = e.monitorUISettings) ||
                                void 0 === M
                                ? void 0
                                : M.generate
                              : C,
                        },
                      };
                    }),
                      void 0 !== v && m(v),
                      void 0 !== d && u(d);
                  }
                },
                [g]
              );
            return (
              Object(o.useEffect)(
                function () {
                  if (s) {
                    console.log(
                      "opening ws connection on localhost:".concat(s)
                    ),
                      (O.current = new WebSocket("ws://localhost:".concat(s))),
                      h(!0);
                    var e = O.current;
                    return function () {
                      console.log("closing ws connection"), e.close();
                    };
                  }
                },
                [s]
              ),
              Object(o.useEffect)(
                function () {
                  p &&
                    O.current &&
                    (O.current.onmessage = function (e) {
                      var t = JSON.parse(e.data);
                      E(t);
                    });
                },
                [p, E]
              ),
              Object(o.useEffect)(
                function () {
                  if (null === n) {
                    var e = new URLSearchParams(window.location.search),
                      t = {
                        pluggableMonitorSettings: {
                          baudrate: {
                            id: "baudrate",
                            label: "Baudrate",
                            type: "enum",
                            values: (e.get("baudrates") || "").split(","),
                            selectedValue: e.get("currentBaudrate") || "9600",
                          },
                        },
                        monitorUISettings: {
                          lineEnding: d(e.get("lineEnding"))
                            ? e.get("lineEnding")
                            : "\n",
                          darkTheme: "true" === e.get("darkTheme"),
                          wsPort: parseInt(e.get("wsPort") || "3030"),
                          interpolate: "true" === e.get("interpolate"),
                          serialPort:
                            e.get("serialPort") || "/serial/port/address",
                          connected: "true" === e.get("connected"),
                          generate: "true" === e.get("generate"),
                        },
                      };
                    E({
                      command:
                        a.Protocol.MiddlewareCommand.ON_SETTINGS_DID_CHANGE,
                      data: t,
                    });
                  }
                },
                [n, E]
              ),
              Object(o.useEffect)(
                function () {
                  var e;
                  if (
                    null == n ||
                    null === (e = n.monitorUISettings) ||
                    void 0 === e
                      ? void 0
                      : e.generate
                  ) {
                    var t = setInterval(function () {
                      var e = (function () {
                        for (var e = [], t = 1; t <= 30; t++)
                          e.push.apply(e, Object(T.a)(D()));
                        return e;
                      })();
                      E(e);
                    }, 32);
                    return function () {
                      clearInterval(t);
                    };
                  }
                },
                [n, E]
              ),
              (n && Object(C.jsx)(y, { config: n, wsSend: x, ref: S })) || null
            );
          }, {}),
        }),
        document.getElementById("root")
      ),
        (function (e) {
          e &&
            e instanceof Function &&
            n
              .e(3)
              .then(n.bind(null, 296))
              .then(function (t) {
                var n = t.getCLS,
                  a = t.getFID,
                  o = t.getFCP,
                  i = t.getLCP,
                  l = t.getTTFB;
                n(e), a(e), o(e), i(e), l(e);
              });
        })();
    },
  },
  [[289, 1, 2]],
]);
